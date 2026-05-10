# Fix: previous run mangled multi-line CJK comments into a single line, which
# accidentally commented out the tagGearFaces() function signature. Replace the
# garbled block with an ASCII-only comment block.

$path = 'E:\Forwok\fastcae_biye\src\GeometryCommand\GeoCommandCreateGear.cpp'
$bytes = [System.IO.File]::ReadAllBytes($path)
$content = [System.Text.Encoding]::UTF8.GetString($bytes)
if ($bytes.Length -ge 3 -and $bytes[0] -eq 0xEF -and $bytes[1] -eq 0xBB -and $bytes[2] -eq 0xBF) {
    $content = $content.TrimStart([char]0xFEFF)
}

$startAnchor = '====================='
$startMarker = 'P1-T07: face semantic tagging ===================='
$startIdx = $content.IndexOf($startMarker)
if ($startIdx -lt 0) { throw 'tag block start not found' }
# Walk back to start of line.
$lineStart = $content.LastIndexOf("`n", $startIdx) + 1

# End: the `void GeoCommandCreateGear::tagGearFaces(Geometry::GeometrySet* set,`
$endAnchor = 'void GeoCommandCreateGear::tagGearFaces(Geometry::GeometrySet* set,'
$endIdx = $content.IndexOf($endAnchor)
if ($endIdx -lt 0) { throw 'tagGearFaces signature not found' }
# Replace from lineStart up to (but not including) endIdx with a clean comment block.

$replacement = "`t// ==================== P1-T07: face semantic tagging ====================`r`n" +
               "`t// Auto-tag faces of a gear GeometrySet for downstream BC application.`r`n" +
               "`t// Cylindrical faces: radius approx holeRadius -> hub_hole;`r`n" +
               "`t//                    radius approx rootRadius -> root_fillet;`r`n" +
               "`t//                    other -> tooth_flank.`r`n" +
               "`t// Planar faces with Z normal: highest Z -> front_face, lowest -> back_face.`r`n" +
               "`t"

$newContent = $content.Substring(0, $lineStart) + $replacement + $content.Substring($endIdx)

# Also fix two other corrupted lines later in the function (CJK comments inside loops).
# These show as "// 鍏堟壂涓€娆?...double zMaxPlane = ..." (comment swallowing newline before code).
$fixes = @(
    @{
        Find = "// 鍏堟壂涓€娆?";
        Repl = "// First pass: locate Z-extreme planar faces for front/back.`r`n`t`t"
    },
    @{
        Find = "// 鏀堕泦";
        Repl = "// Collect (faceId, face) pairs, indexed by TopExp_Explorer order.`r`n`t`t"
    },
    @{
        Find = "// 榻块《鍦嗘煴闈?";
        Repl = "// Tip cylinder or other special cylinders also fall into tooth_flank.`r`n`t`t`t`t`t"
    },
    @{
        Find = "// BSpline / Bezier / Conic";
        Repl = "// BSpline / Bezier / Conic / other -> tooth_flank.`r`n`t`t`t`t"
    }
)

foreach ($fix in $fixes) {
    # Match the start of the bad comment up to the next code statement (often something like "double", "QList", "set->", etc.)
    # Use regex to chomp from the comment marker through the rest of the corrupted line up to a code identifier.
    $pattern = [regex]::Escape($fix.Find) + '[^\r\n]*?(?=(double |QList |set->))'
    $newContent = [regex]::Replace($newContent, $pattern, $fix.Repl)
}

# Final pass: normalize CRLF.
$newContent = $newContent -replace "`r`n", "`n"
$newContent = $newContent -replace "`n", "`r`n"

$utf8Bom = New-Object System.Text.UTF8Encoding($true)
[System.IO.File]::WriteAllText($path, $newContent, $utf8Bom)

Write-Output ('Fixed comment block. New file size: ' + ([System.IO.File]::ReadAllBytes($path).Length))
