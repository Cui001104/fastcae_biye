# P1-T04 refactor: replace buildGearProfileWire body and createSecondGearProfile body.
# Reads the .cpp file, locates each function by signature, brace-counts to find the
# matching close brace, and splices in clean parameterized bodies.

$path = 'E:\Forwok\fastcae_biye\src\GeometryCommand\GeoCommandCreateGear.cpp'

# ---- Read file as UTF-8 string, preserve BOM detection ----
$bytes = [System.IO.File]::ReadAllBytes($path)
$hasBom = $bytes.Length -ge 3 -and $bytes[0] -eq 0xEF -and $bytes[1] -eq 0xBB -and $bytes[2] -eq 0xBF
$content = [System.Text.Encoding]::UTF8.GetString($bytes)
if ($hasBom) { $content = $content.TrimStart([char]0xFEFF) }

function Find-FunctionBodyRange {
    param([string]$src, [string]$signature)
    $idx = $src.IndexOf($signature)
    if ($idx -lt 0) { throw "signature not found: $signature" }
    # Find first '{' after signature
    $openIdx = $src.IndexOf('{', $idx)
    if ($openIdx -lt 0) { throw "open brace not found after $signature" }
    # Brace-count to matching '}'
    $depth = 1
    $i = $openIdx + 1
    while ($i -lt $src.Length -and $depth -gt 0) {
        $c = $src[$i]
        if ($c -eq '{') { $depth++ }
        elseif ($c -eq '}') { $depth-- }
        $i++
    }
    if ($depth -ne 0) { throw "unbalanced braces for $signature" }
    return @{ OpenIdx = $openIdx; CloseIdx = $i - 1 }  # CloseIdx points to '}'
}

# ---- New body for buildGearProfileWire ----
$newWireBody = @'
{
		// Generic gear profile wire builder (parameterized version of original gear-1 path).
		// Z      : own gear tooth count
		// Zmate  : mating gear tooth count (used in working center distance)
		// xOwn   : this gear's profile shift coefficient
		// xSum   : x1 + x2 (shared between both gears)
		// tipReliefAmount, tipReliefLength : own gear's parabolic tip relief
		// center : final placement of the wire (origin for gear 1, (0, a', 0) for gear 2)
		const double m   = _module;
		const double phi = _pressureAngle * M_PI / 180.0;
		const int    Z2  = Zmate;            // alias: legacy expressions reference Z2
		const double x1  = xOwn;             // alias
		const double x2  = xSum - xOwn;      // alias (x1 + x2 == xSum)

		double centerDistance;
		double y_delt = 0.0;
		double alphaPrime = phi;

		if (xSum == 0.0) {
			centerDistance = (Z + Z2) * m / 2.0;
		} else {
			const double a            = (Z + Z2) * m / 2.0;
			const double x_sig        = xSum;
			const double invAlpha     = std::tan(phi) - phi;
			const double invAlphaPrime = invAlpha + 2 * x_sig * std::tan(phi) / (Z + Z2);

			alphaPrime = phi;
			const double tolerance = 1e-10;
			const int    maxIterations = 100;
			for (int i = 0; i < maxIterations; ++i) {
				const double f      = std::tan(alphaPrime) - alphaPrime - invAlphaPrime;
				const double fPrime = 1.0 / (std::cos(alphaPrime) * std::cos(alphaPrime)) - 1.0;
				const double delta  = f / fPrime;
				alphaPrime -= delta;
				if (std::abs(delta) < tolerance) break;
			}

			centerDistance = a * std::cos(phi) / std::cos(alphaPrime);
			const double y = (centerDistance - a) / m;
			y_delt = x_sig - y;
		}
		(void)centerDistance;  // not used directly inside helper (gear 2 wrapper computes its own)
		(void)alphaPrime;

		const double Rref = Z * m / 2.0;
		double ha = _addendumCoeff * m;
		if (xSum != 0.0) ha = (_addendumCoeff + x1 - y_delt) * m;
		const double Rb = Rref * std::cos(phi);
		const double Ra = Rref + ha;

		double hf = _dedendumCoeff * m;
		if (xSum != 0.0) hf = (_dedendumCoeff - x1) * m;
		double Rf = Rref - hf;
		if (Rf < 0) Rf = 0.1 * m;

		const double angularPitch = 2.0 * M_PI / Z;
		double toothThicknessHalfAngle;
		if (xSum == 0.0) {
			toothThicknessHalfAngle = angularPitch / 4.0;
		} else {
			const double s = m * (M_PI / 2.0 + 2 * x1 * std::tan(phi));
			toothThicknessHalfAngle = s / (2.0 * Rref);
		}

		double thetaStart = 0.0;
		if (Rf > Rb) thetaStart = std::sqrt(Rf * Rf - Rb * Rb) / Rb;
		const double thetaEnd = std::sqrt(Ra * Ra - Rb * Rb) / Rb;

		const int numPoints = 20;
		std::vector<gp_Pnt> involuteLeft;
		std::vector<gp_Pnt> involuteRight;
		involuteLeft.reserve(numPoints + 1);
		involuteRight.reserve(numPoints + 1);

		const double R_relief_start = Ra - tipReliefLength;

		for (int i = 0; i <= numPoints; ++i) {
			const double t     = (double)i / numPoints;
			const double theta = thetaStart + t * (thetaEnd - thetaStart);
			gp_Pnt pt          = involutePoint(Rb, theta);

			const double R_current = std::sqrt(pt.X() * pt.X() + pt.Y() * pt.Y());

			// Parabolic tip relief: shift inward by Ca * (y/Lca)^2 along the radial direction.
			if (tipReliefAmount > 0.0 && tipReliefLength > 0.0 && R_current > R_relief_start) {
				const double yRel  = R_current - R_relief_start;
				const double delta = tipReliefAmount * (yRel / tipReliefLength) * (yRel / tipReliefLength);
				const double nx    = pt.X() / R_current;
				const double ny    = pt.Y() / R_current;
				pt.SetX(pt.X() - delta * nx);
				pt.SetY(pt.Y() - delta * ny);
			}

			const double angleAtRef = involuteAngle(Rb, Rref);
			gp_Pnt ptRotated = rotatePoint(pt, -angleAtRef - toothThicknessHalfAngle);
			involuteLeft.push_back(ptRotated);
			involuteRight.push_back(mirrorPoint(ptRotated));
		}

		BRepBuilderAPI_MakeWire wireBuilder;

		for (int tooth = 0; tooth < Z; ++tooth) {
			const double toothAngle = tooth * angularPitch;

			// Left flank: root -> tip
			for (size_t i = 0; i + 1 < involuteLeft.size(); ++i) {
				gp_Pnt p1 = rotatePoint(involuteLeft[i], toothAngle);
				gp_Pnt p2 = rotatePoint(involuteLeft[i + 1], toothAngle);
				if (p1.Distance(p2) > 1e-6)
					wireBuilder.Add(BRepBuilderAPI_MakeEdge(p1, p2));
			}

			// Tip arc through tipLeft, tipMid (on Ra circle), tipRight.
			gp_Pnt tipLeft  = rotatePoint(involuteLeft.back(),  toothAngle);
			gp_Pnt tipRight = rotatePoint(involuteRight.back(), toothAngle);
			if (tipLeft.Distance(tipRight) > 1e-6) {
				gp_Pnt tipMid((tipLeft.X() + tipRight.X()) / 2.0 * Ra
				                  / std::sqrt(std::pow((tipLeft.X() + tipRight.X()) / 2.0, 2)
				                              + std::pow((tipLeft.Y() + tipRight.Y()) / 2.0, 2)),
				              (tipLeft.Y() + tipRight.Y()) / 2.0 * Ra
				                  / std::sqrt(std::pow((tipLeft.X() + tipRight.X()) / 2.0, 2)
				                              + std::pow((tipLeft.Y() + tipRight.Y()) / 2.0, 2)),
				              0);
				try {
					GC_MakeArcOfCircle arcMaker(tipLeft, tipMid, tipRight);
					if (arcMaker.IsDone())
						wireBuilder.Add(BRepBuilderAPI_MakeEdge(arcMaker.Value()));
					else
						wireBuilder.Add(BRepBuilderAPI_MakeEdge(tipLeft, tipRight));
				} catch (...) {
					wireBuilder.Add(BRepBuilderAPI_MakeEdge(tipLeft, tipRight));
				}
			}

			// Right flank: tip -> root
			for (int i = (int)involuteRight.size() - 1; i > 0; --i) {
				gp_Pnt p1 = rotatePoint(involuteRight[i], toothAngle);
				gp_Pnt p2 = rotatePoint(involuteRight[i - 1], toothAngle);
				if (p1.Distance(p2) > 1e-6)
					wireBuilder.Add(BRepBuilderAPI_MakeEdge(p1, p2));
			}

			// Root arc bridging current right-root to next-tooth left-root.
			gp_Pnt rootRight    = rotatePoint(involuteRight.front(), toothAngle);
			gp_Pnt nextRootLeft = rotatePoint(involuteLeft.front(),  toothAngle + angularPitch);
			if (rootRight.Distance(nextRootLeft) > 1e-6) {
				double rootRightAngle    = std::atan2(rootRight.Y(),    rootRight.X());
				double nextRootLeftAngle = std::atan2(nextRootLeft.Y(), nextRootLeft.X());
				if (nextRootLeftAngle < rootRightAngle) nextRootLeftAngle += 2.0 * M_PI;
				const double rootArcSpan = nextRootLeftAngle - rootRightAngle;

				if (rootArcSpan > M_PI) {
					// > 180 degrees: cap at 180 deg arc and bridge the rest with a line.
					const double limitedEndAngle = rootRightAngle + M_PI;
					const double rootMidAngle    = rootRightAngle + M_PI / 2.0;
					gp_Pnt rootMid(Rf * std::cos(rootMidAngle), Rf * std::sin(rootMidAngle), 0);
					gp_Pnt arcEnd(Rf * std::cos(limitedEndAngle), Rf * std::sin(limitedEndAngle), 0);
					try {
						GC_MakeArcOfCircle arcMaker(rootRight, rootMid, arcEnd);
						if (arcMaker.IsDone())
							wireBuilder.Add(BRepBuilderAPI_MakeEdge(arcMaker.Value()));
					} catch (...) {}
					if (arcEnd.Distance(nextRootLeft) > 1e-6)
						wireBuilder.Add(BRepBuilderAPI_MakeEdge(arcEnd, nextRootLeft));
				} else {
					const double minRootArcSpan = 1e-6;
					if (rootArcSpan < minRootArcSpan) {
						wireBuilder.Add(BRepBuilderAPI_MakeEdge(rootRight, nextRootLeft));
					} else {
						const double rootMidAngle = (rootRightAngle + nextRootLeftAngle) / 2.0;
						gp_Pnt rootMid(Rf * std::cos(rootMidAngle), Rf * std::sin(rootMidAngle), 0);
						const double det = rootRight.X() * (rootMid.Y() - nextRootLeft.Y()) +
						                   rootMid.X() * (nextRootLeft.Y() - rootRight.Y()) +
						                   nextRootLeft.X() * (rootRight.Y() - rootMid.Y());
						const bool useCircleMethod = (qAbs(det) < 0.1);
						bool added = false;
						try {
							if (useCircleMethod) {
								gp_Circ circle(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), Rf);
								GC_MakeArcOfCircle arcMaker(circle, rootRightAngle, nextRootLeftAngle, true);
								if (arcMaker.IsDone()) {
									wireBuilder.Add(BRepBuilderAPI_MakeEdge(arcMaker.Value()));
									added = true;
								}
							} else {
								GC_MakeArcOfCircle arcMaker(rootRight, rootMid, nextRootLeft);
								if (arcMaker.IsDone()) {
									wireBuilder.Add(BRepBuilderAPI_MakeEdge(arcMaker.Value()));
									added = true;
								} else {
									gp_Circ circle(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), Rf);
									GC_MakeArcOfCircle arcMaker2(circle, rootRightAngle, nextRootLeftAngle, true);
									if (arcMaker2.IsDone()) {
										wireBuilder.Add(BRepBuilderAPI_MakeEdge(arcMaker2.Value()));
										added = true;
									}
								}
							}
						} catch (...) {}
						if (!added) {
							try {
								gp_Circ circle(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), Rf);
								GC_MakeArcOfCircle arcMaker(circle, rootRightAngle, nextRootLeftAngle, true);
								if (arcMaker.IsDone())
									wireBuilder.Add(BRepBuilderAPI_MakeEdge(arcMaker.Value()));
								else
									wireBuilder.Add(BRepBuilderAPI_MakeEdge(rootRight, nextRootLeft));
							} catch (...) {
								wireBuilder.Add(BRepBuilderAPI_MakeEdge(rootRight, nextRootLeft));
							}
						}
					}
				}
			}
		}

		if (!wireBuilder.IsDone()) {
			qDebug() << "buildGearProfileWire: wire not done, Z =" << Z;
			return TopoDS_Wire();
		}

		TopoDS_Wire wire;
		try {
			wire = wireBuilder.Wire();
		} catch (Standard_Failure& e) {
			qDebug() << "buildGearProfileWire: wire exception:" << e.GetMessageString();
			return TopoDS_Wire();
		}

		if (center.X() != 0.0 || center.Y() != 0.0 || center.Z() != 0.0) {
			gp_Trsf trsf;
			trsf.SetTranslation(gp_Vec(center.X(), center.Y(), center.Z()));
			BRepBuilderAPI_Transform tf(wire, trsf);
			wire = TopoDS::Wire(tf.Shape());
		}
		return wire;
	}
'@

# ---- New body for createSecondGearProfile (thin wrapper) ----
$newSecondBody = @'
{
		// Thin wrapper. Geometry built by buildGearProfileWire then translated to (0, a', 0).
		const double cd = centerDistanceBetweenGears();
		return buildGearProfileWire(_numberOfSecondTeeth, _numberOfTeeth,
		                            _x2, _x1 + _x2,
		                            _tipReliefAmount2, _tipReliefLength2,
		                            gp_Pnt(0, cd, 0));
	}
'@

# ---- Splice ----

# 1. buildGearProfileWire body
$wireSig = 'TopoDS_Wire GeoCommandCreateGear::buildGearProfileWire('
$range = Find-FunctionBodyRange -src $content -signature $wireSig
$content = $content.Substring(0, $range.OpenIdx) + $newWireBody + $content.Substring($range.CloseIdx + 1)
Write-Output ('Replaced buildGearProfileWire body, OpenIdx=' + $range.OpenIdx + ' CloseIdx=' + $range.CloseIdx)

# 2. createSecondGearProfile body (search in updated content)
$secondSig = 'TopoDS_Wire GeoCommandCreateGear::createSecondGearProfile()'
$range2 = Find-FunctionBodyRange -src $content -signature $secondSig
$content = $content.Substring(0, $range2.OpenIdx) + $newSecondBody + $content.Substring($range2.CloseIdx + 1)
Write-Output ('Replaced createSecondGearProfile body, OpenIdx=' + $range2.OpenIdx + ' CloseIdx=' + $range2.CloseIdx)

# ---- Write back as UTF-8 with BOM ----
$utf8Bom = New-Object System.Text.UTF8Encoding($true)
[System.IO.File]::WriteAllText($path, $content, $utf8Bom)
Write-Output 'Wrote updated file.'

# ---- Sanity: count lines ----
$newBytes = [System.IO.File]::ReadAllBytes($path)
$newText = [System.Text.Encoding]::UTF8.GetString($newBytes)
$lineCount = ($newText -split "`r`n").Count
Write-Output ('New file line count: ' + $lineCount)
