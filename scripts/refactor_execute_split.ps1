# P1-T05 + P1-T07: rewrite the Fuse-and-emit block in execute() to output two
# independent GeometrySets and tag faces.

$path = 'E:\Forwok\fastcae_biye\src\GeometryCommand\GeoCommandCreateGear.cpp'
$bytes = [System.IO.File]::ReadAllBytes($path)
$hasBom = $bytes.Length -ge 3 -and $bytes[0] -eq 0xEF -and $bytes[1] -eq 0xBB -and $bytes[2] -eq 0xBF
$content = [System.Text.Encoding]::UTF8.GetString($bytes)
if ($hasBom) { $content = $content.TrimStart([char]0xFEFF) }

# Locate the Fuse block by ASCII anchor (signatures and brace positions).
$startAnchor = 'BRepAlgoAPI_Fuse fuseMaker(gearShape, secondGearShape);'
$startIdx = $content.IndexOf($startAnchor)
if ($startIdx -lt 0) { throw "Fuse anchor not found" }
# Walk backwards to the start of the line (so we replace from the leading tabs).
$lineStart = $content.LastIndexOf("`n", $startIdx) + 1

# Locate the end: the closing `}` of the execute() function.
# After the Fuse block, execute() proceeds with `_res = set;`, parameter setup,
# `emit showSet(set);`, `return true;`, then `}`. We replace everything from
# lineStart up to and including the line `return true;` (we keep the trailing `}` of execute()).
$returnAnchor = 'return true;'
$returnIdx = $content.IndexOf($returnAnchor, $startIdx)
if ($returnIdx -lt 0) { throw "return true; anchor not found after Fuse" }
# Find end of that line
$returnLineEnd = $content.IndexOf("`n", $returnIdx)
if ($returnLineEnd -lt 0) { throw "newline after return true not found" }
# We replace [lineStart, returnLineEnd] inclusive of the newline.
$replaceStart = $lineStart
$replaceEnd   = $returnLineEnd + 1   # exclusive

$old = $content.Substring($replaceStart, $replaceEnd - $replaceStart)

$new = @'
		// P1-T05: 不再 Fuse 两齿轮，作为两个独立 GeometrySet 输出，便于后续 BC 自动加载。
		TopoDS_Shape* shape1 = new TopoDS_Shape;
		*shape1 = gearShape;
		TopoDS_Shape* shape2 = new TopoDS_Shape;
		*shape2 = secondGearShape;

		Geometry::GeometrySet* set1 = new Geometry::GeometrySet(Geometry::STEP);
		set1->setShape(shape1);
		Geometry::GeometrySet* set2 = new Geometry::GeometrySet(Geometry::STEP);
		set2->setShape(shape2);

		_res  = set1;
		_res2 = set2;

		// P1-T07: 自动给每个 Set 打面语义标签。
		const double Rf1 = _module * _numberOfTeeth        / 2.0 - _dedendumCoeff * _module;
		const double Rf2 = _module * _numberOfSecondTeeth  / 2.0 - _dedendumCoeff * _module;
		tagGearFaces(set1, holeRadius,  Rf1 > 0 ? Rf1 : 0.1 * _module);
		tagGearFaces(set2, holeRadius2, Rf2 > 0 ? Rf2 : 0.1 * _module);

		const QString baseName = _name.isEmpty() ? QStringLiteral("Gear") : _name;
		const QString name1    = baseName + QStringLiteral("_1");
		const QString name2    = baseName + QStringLiteral("_2");

		if(_isEdit) {
			set1->setName(_editSet->getName());
			_geoData->replaceSet(set1, _editSet);
			emit removeDisplayActor(_editSet);
			set2->setName(name2);
			_geoData->appendGeometrySet(set2);
		} else {
			set1->setName(name1);
			set2->setName(name2);
			_geoData->appendGeometrySet(set1);
			_geoData->appendGeometrySet(set2);
		}

		// 创建参数对象，挂在主齿轮 Set 上。
		Geometry::GeometryParaGear* para = new Geometry::GeometryParaGear;
		para->setName(_name);
		para->setNumberOfTeeth(_numberOfTeeth);
		para->setNumberOfSecondTeeth(_numberOfSecondTeeth);
		para->setModule(_module);
		para->setPressureAngle(_pressureAngle);
		para->setAddendumCoefficient(_addendumCoeff);
		para->setDedendumCoefficient(_dedendumCoeff);
		para->setFilletCoefficient(_filletCoeff);
		para->setThickness(_thickness);
		para->setExternalGear(_externalGear);
		para->setTipReliefAmount(_tipReliefAmount);
		para->setTipReliefLength(_tipReliefLength);
		para->setTipReliefAmount2(_tipReliefAmount2);
		para->setTipReliefLength2(_tipReliefLength2);
		para->setProfileShiftCoefficient1(_x1);
		para->setProfileShiftCoefficient2(_x2);
		_res->setParameter(para);

		GeoCommandBase::execute();
		emit updateGeoTree();
		emit showSet(set1);
		emit showSet(set2);

		return true;
'@

$newContent = $content.Substring(0, $replaceStart) + $new + $content.Substring($replaceEnd)

# Add tagGearFaces implementation just before the closing namespace.
$tagImpl = @'

	// ==================== P1-T07: face semantic tagging ====================
	// 给一个齿轮 GeometrySet 自动打面语义标签。
	// 圆柱面：半径 ≈ holeRadius → "hub_hole"；
	//        半径 ≈ rootRadius → "root_fillet"；
	//        其它（齿廓 + 齿顶圆）→ "tooth_flank"。
	// 平面：Z 法向 → "front_face" (Z 较大) / "back_face" (Z 较小)。
	void GeoCommandCreateGear::tagGearFaces(Geometry::GeometrySet* set,
	                                        double holeRadius,
	                                        double rootRadius)
	{
		if(!set) return;
		TopoDS_Shape* shapePtr = set->getShape();
		if(!shapePtr || shapePtr->IsNull()) return;
		const TopoDS_Shape& shape = *shapePtr;

		// 先扫一次：定位 Z 最大、最小的平面，用于 front/back 区分。
		double zMaxPlane = -std::numeric_limits<double>::infinity();
		double zMinPlane =  std::numeric_limits<double>::infinity();
		int    frontId = -1, backId = -1;

		// 收集 (faceId, face) 对，按 TopExp_Explorer 顺序赋 id。
		QList<QPair<int, TopoDS_Face>> faces;
		int idx = 0;
		for(TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next(), ++idx) {
			TopoDS_Face f = TopoDS::Face(exp.Current());
			faces.append(qMakePair(idx, f));
		}

		const double tol = 1e-3;
		for(const auto& pr : faces) {
			const int faceId    = pr.first;
			const TopoDS_Face f = pr.second;
			BRepAdaptor_Surface surf(f, Standard_True);
			GeomAbs_SurfaceType st = surf.GetType();

			if(st == GeomAbs_Plane) {
				gp_Pln pln = surf.Plane();
				gp_Dir n   = pln.Axis().Direction();
				if(std::abs(n.Z()) > 0.99) {
					Bnd_Box bb;
					BRepBndLib::Add(f, bb);
					double xmin, ymin, zmin, xmax, ymax, zmax;
					bb.Get(xmin, ymin, zmin, xmax, ymax, zmax);
					double zMid = 0.5 * (zmin + zmax);
					if(zMid > zMaxPlane) { zMaxPlane = zMid; frontId = faceId; }
					if(zMid < zMinPlane) { zMinPlane = zMid; backId  = faceId; }
				}
			} else if(st == GeomAbs_Cylinder) {
				gp_Cylinder cyl = surf.Cylinder();
				double r = cyl.Radius();
				if(std::abs(r - holeRadius) < tol * std::max(holeRadius, 1.0)) {
					set->setSemanticTag(faceId, QStringLiteral("hub_hole"));
				} else if(std::abs(r - rootRadius) < tol * std::max(rootRadius, 1.0)) {
					set->setSemanticTag(faceId, QStringLiteral("root_fillet"));
				} else {
					// 齿顶圆柱面或其它特殊柱面也归到 tooth_flank（齿啮合时受力的面）。
					set->setSemanticTag(faceId, QStringLiteral("tooth_flank"));
				}
			} else {
				// BSpline / Bezier / Conic / 其它：齿廓
				set->setSemanticTag(faceId, QStringLiteral("tooth_flank"));
			}
		}
		if(frontId >= 0) set->setSemanticTag(frontId, QStringLiteral("front_face"));
		if(backId  >= 0 && backId != frontId)
			set->setSemanticTag(backId, QStringLiteral("back_face"));

		qDebug() << "tagGearFaces: total faces =" << faces.size()
		         << ", hub_hole =" << set->getFacesByTag("hub_hole").size()
		         << ", root_fillet =" << set->getFacesByTag("root_fillet").size()
		         << ", tooth_flank =" << set->getFacesByTag("tooth_flank").size()
		         << ", front/back =" << (frontId >= 0) << "/" << (backId >= 0);
	}

'@

# Insert tagImpl before the trailing `} // namespace Command` line.
$nsCloseAnchor = '} // namespace Command'
$nsIdx = $newContent.IndexOf($nsCloseAnchor)
if ($nsIdx -lt 0) { throw "namespace close anchor not found" }
$newContent = $newContent.Substring(0, $nsIdx) + $tagImpl + $newContent.Substring($nsIdx)

# Need <limits> for numeric_limits.
if ($newContent -notmatch '#include <limits>') {
    $newContent = $newContent -replace '#include <cmath>', "#include <cmath>`r`n#include <limits>"
}

# Normalize CRLF
$newContent = $newContent -replace "`r`n", "`n"
$newContent = $newContent -replace "`n", "`r`n"

$utf8Bom = New-Object System.Text.UTF8Encoding($true)
[System.IO.File]::WriteAllText($path, $newContent, $utf8Bom)

Write-Output ('Replaced execute() block, replaceStart=' + $replaceStart + ' replaceEnd=' + $replaceEnd + ' lengthDelta=' + ($new.Length - $old.Length))
Write-Output ('Inserted tagGearFaces impl at position ' + $nsIdx)
Write-Output ('Final byte length: ' + ([System.IO.File]::ReadAllBytes($path).Length))
