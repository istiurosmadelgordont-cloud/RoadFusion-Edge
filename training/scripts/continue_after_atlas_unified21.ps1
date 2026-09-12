param(
    [int]$Epochs = 24,
    [int]$Batch = 16,
    [switch]$Shutdown
)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$python = Join-Path $root '.venv-train\Scripts\python.exe'
$atlasStatus = Join-Path $root 'reports\atlas_pipeline_status.json'
$status = Join-Path $root 'reports\unified21_auto_pipeline_status.json'
$atlasDataset = Join-Path $root 'datasets\atlas_full_eight_lights'
$reviewedDataset = Join-Path $root 'datasets\six_light_reviewed_additions_masked'
$v4Dataset = Join-Path $root 'datasets\unified21_light_focus_v4'
$initialized = Join-Path $root 'weights\unified21_light_focus_v4_initialized.pt'
$names = @(
    'pedestrian','rider','car','bus','truck','motorcycle','bicycle',
    'traffic_red_circle','traffic_red_left','traffic_red_right','traffic_red_straight',
    'traffic_green_circle','traffic_green_left','traffic_green_right','traffic_green_straight',
    'traffic_sign','crosswalk','guide_arrows','traffic_cone','roadworks_sign','delineator'
)
$lights = @(
    'traffic_red_circle','traffic_red_left','traffic_red_right','traffic_red_straight',
    'traffic_green_circle','traffic_green_left','traffic_green_right','traffic_green_straight'
)

function Write-Status([string]$state, [string]$detail, $extra = @{}) {
    $doc = [ordered]@{ state=$state; detail=$detail; updated_at=(Get-Date).ToString('o'); pid=$PID; single_model=$true; class_count=21 }
    foreach ($entry in $extra.GetEnumerator()) { $doc[$entry.Key] = $entry.Value }
    $doc | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $status -Encoding UTF8
}

try {
    Write-Status 'waiting_for_atlas' 'Waiting for public ATLAS still images; local D-drive videos are excluded.'
    while ($true) {
        if (-not (Test-Path -LiteralPath $atlasStatus)) { Start-Sleep -Seconds 30; continue }
        $atlas = Get-Content -Raw -LiteralPath $atlasStatus | ConvertFrom-Json
        if ($atlas.state -eq 'failed') { throw "ATLAS pipeline failed: $($atlas.detail)" }
        if ($atlas.state -eq 'ready') { break }
        Start-Sleep -Seconds 30
    }

    $reportPath = Join-Path $atlasDataset 'report.json'
    $report = Get-Content -Raw -LiteralPath $reportPath | ConvertFrom-Json
    $deficits = [ordered]@{}
    foreach ($name in $lights) {
        $trainKey = "train:$name"
        $valKey = "val:$name"
        $trainHave = [int]($report.selected_anchors.$trainKey)
        $valHave = [int]($report.selected_anchors.$valKey)
        if ($trainHave -lt 1000) { $deficits[$trainKey] = 1000 - $trainHave }
        if ($valHave -lt 150) { $deficits[$valKey] = 150 - $valHave }
    }
    if ($deficits.Count -gt 0) {
        Write-Status 'needs_more_data' 'Eight-light real-data or independent-validation thresholds are not met; training was not started.' @{ deficits=$deficits; report=$reportPath }
        exit 2
    }

    Write-Status 'quality_control' 'Counts passed; rendering balanced eight-light QA sheets.' @{ report=$reportPath }
    foreach ($split in @('train','val')) {
        foreach ($classId in 7..14) {
            & $python (Join-Path $root 'scripts\render_yolo_contact_sheet.py') $atlasDataset `
                --split $split --class-id $classId --limit 36 `
                --output (Join-Path $root "reports\atlas_full_${split}_class_${classId}.jpg")
            if ($LASTEXITCODE -ne 0) { throw "Contact sheet failed: split=$split class=$classId" }
        }
    }

    if (-not (Test-Path -LiteralPath $initialized)) {
        & $python (Join-Path $root 'scripts\initialize_unified21_v4.py')
        if ($LASTEXITCODE -ne 0) { throw 'Unified19-to-Unified21 initialization failed.' }
    }

    $externalConfig = Join-Path $root 'configs\atlas_full_eight_lights.yaml'
    $atlasYamlPath = $atlasDataset.Replace('\','/')
    $yaml = "path: $atlasYamlPath`ntrain: images/train`nval: images/val`nnames:`n"
    for ($i=0; $i -lt $names.Count; $i++) { $yaml += "  $i`: $($names[$i])`n" }
    $yaml | Set-Content -LiteralPath $externalConfig -Encoding UTF8

    Write-Status 'external_baseline' 'Evaluating the migrated 21-class baseline on ATLAS test.'
    & $python (Join-Path $root 'scripts\evaluate_detector.py') $initialized --data $externalConfig --tag atlas_full_unified21_baseline
    if ($LASTEXITCODE -ne 0) { throw 'ATLAS Unified21 baseline evaluation failed.' }

    if (Test-Path -LiteralPath $v4Dataset) { throw "Refusing to overwrite existing dataset: $v4Dataset" }
    Write-Status 'building_dataset' 'Building one 21-class dataset with circle and straight arrows separated.'
    & $python (Join-Path $root 'scripts\build_unified21_light_focus_v4.py') `
        --old19-addition $reviewedDataset --new21-addition $atlasDataset
    if ($LASTEXITCODE -ne 0) { throw 'Unified21 dataset build failed.' }
    & $python (Join-Path $root 'scripts\train.py') --task unified21lightv4 --check-only
    if ($LASTEXITCODE -ne 0) { throw 'Unified21 dataset integrity check failed.' }

    $config = Join-Path $root 'configs\unified21_light_focus_v4.yaml'
    Write-Status 'all_class_baseline' 'Evaluating the migrated model on the final 21-class validation mixture.'
    & $python (Join-Path $root 'scripts\evaluate_detector.py') $initialized --data $config --tag unified21_v4_validation_baseline
    if ($LASTEXITCODE -ne 0) { throw 'Unified21 all-class baseline evaluation failed.' }

    Write-Status 'training' 'Training one 21-class model after all eight light classes passed count gates.' @{ epochs=$Epochs; batch=$Batch }
    & (Join-Path $root 'scripts\train_unified21_light_v4.ps1') -Epochs $Epochs -Batch $Batch
    if ($LASTEXITCODE -ne 0) { throw 'Unified21 V4 training failed.' }

    $candidate = Join-Path $root 'weights\unified21_light_focus_v4_candidate_640.pt'
    Write-Status 'external_evaluation' 'Evaluating the candidate on independent ATLAS test images.'
    & $python (Join-Path $root 'scripts\evaluate_detector.py') $candidate --data $externalConfig --tag atlas_full_unified21_candidate
    if ($LASTEXITCODE -ne 0) { throw 'Unified21 external candidate evaluation failed.' }

    $externalBefore = Get-Content -Raw -LiteralPath (Join-Path $root 'reports\atlas_full_unified21_baseline_evaluation.json') | ConvertFrom-Json
    $externalAfter = Get-Content -Raw -LiteralPath (Join-Path $root 'reports\atlas_full_unified21_candidate_evaluation.json') | ConvertFrom-Json
    $allBefore = Get-Content -Raw -LiteralPath (Join-Path $root 'reports\unified21_v4_validation_baseline_evaluation.json') | ConvertFrom-Json
    $allAfter = Get-Content -Raw -LiteralPath (Join-Path $root 'reports\unified21_light_focus_v4_final_evaluation.json') | ConvertFrom-Json
    $nonLights = $names | Where-Object { $_ -notin $lights }
    $beforeNonLight = ($nonLights | ForEach-Object { [double]$allBefore.per_class.$_.map50 } | Measure-Object -Average).Average
    $afterNonLight = ($nonLights | ForEach-Object { [double]$allAfter.per_class.$_.map50 } | Measure-Object -Average).Average
    $lightImproved = [double]$externalAfter.directional_light_macro_map50 -gt [double]$externalBefore.directional_light_macro_map50
    $overallProtected = [double]$allAfter.map50 -ge ([double]$allBefore.map50 - 0.01)
    $nonLightProtected = [double]$afterNonLight -ge ([double]$beforeNonLight - 0.015)
    $metrics = @{
        external_eight_light_before=[double]$externalBefore.directional_light_macro_map50
        external_eight_light_after=[double]$externalAfter.directional_light_macro_map50
        all21_before=[double]$allBefore.map50; all21_after=[double]$allAfter.map50
        non_light_macro_before=[double]$beforeNonLight; non_light_macro_after=[double]$afterNonLight
        light_improved=$lightImproved; overall_protected=$overallProtected; non_light_protected=$nonLightProtected
    }
    if ($lightImproved -and $overallProtected -and $nonLightProtected) {
        $selected = Join-Path $root 'weights\unified21_light_focus_v4_selected_640.pt'
        Copy-Item -LiteralPath $candidate -Destination $selected -Force
        $metrics.selected_model = $selected
        Write-Status 'completed' 'Unified21 improved eight-light accuracy while protecting the other classes.' $metrics
    } else {
        Write-Status 'completed_not_selected' 'Candidate failed one or more accuracy gates; the initialized baseline remains available.' $metrics
    }

    if ($Shutdown) { shutdown.exe /s /t 120 /c 'RoadFusion Edge Unified21 pipeline completed. Run shutdown.exe /a within 120 seconds to cancel.' }
}
catch {
    Write-Status 'failed' $_.Exception.Message
    throw
}
