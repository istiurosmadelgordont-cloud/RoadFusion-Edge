param(
    [int]$Epochs = 24,
    [int]$Batch = 16,
    [switch]$Shutdown
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$python = Join-Path $projectRoot '.venv-train\Scripts\python.exe'
$trainer = Join-Path $projectRoot 'scripts\train.py'
$evaluator = Join-Path $projectRoot 'scripts\evaluate_detector.py'
$config = Join-Path $projectRoot 'configs\unified21_light_focus_v4.yaml'
$startWeights = Join-Path $projectRoot 'weights\unified21_light_focus_v4_initialized.pt'
$runName = 'unified21_light_focus_v4'
$best = Join-Path $projectRoot "runs\$runName\weights\best.pt"
$candidate = Join-Path $projectRoot 'weights\unified21_light_focus_v4_candidate_640.pt'
$completion = Join-Path $projectRoot 'reports\unified21_light_v4_completion.json'

Set-Location -LiteralPath $projectRoot
$env:YOLO_CONFIG_DIR = Join-Path $projectRoot 'runtime_v21\yolo'
$env:MPLCONFIGDIR = Join-Path $projectRoot 'runtime_v21\matplotlib'
$env:TEMP = Join-Path $projectRoot '.runtime_cache\tmp'
$env:TMP = $env:TEMP
$env:TORCH_HOME = Join-Path $projectRoot '.runtime_cache\torch'
$env:HF_HOME = Join-Path $projectRoot '.runtime_cache\huggingface'
$env:PIP_CACHE_DIR = Join-Path $projectRoot '.runtime_cache\pip'

& $python $trainer --task unified21lightv4 --epochs $Epochs --batch $Batch --imgsz 640 `
    --workers 2 --patience 8 --weights $startWeights --lr0 0.00015 --lrf 0.08 `
    --optimizer AdamW --warmup-epochs 1 --warmup-bias-lr 0.01 --close-mosaic 6 `
    --mosaic 0.20 --hsv-s 0.12 --hsv-v 0.12 --freeze 0 --save-period 2 --name $runName
if ($LASTEXITCODE -ne 0 -or -not (Test-Path -LiteralPath $best)) {
    throw 'Unified21 light-focus V4 training failed.'
}

Copy-Item -LiteralPath $best -Destination $candidate -Force
& $python $evaluator $candidate --data $config --tag unified21_light_focus_v4_final
if ($LASTEXITCODE -ne 0) { throw 'Unified21 light-focus V4 evaluation failed.' }

[ordered]@{
    state = 'completed'
    completed_at = (Get-Date).ToString('o')
    model = $candidate
    evaluation = (Join-Path $projectRoot 'reports\unified21_light_focus_v4_final_evaluation.json')
    single_model = $true
    class_count = 21
    input_size = 640
} | ConvertTo-Json | Set-Content -LiteralPath $completion -Encoding UTF8

if ($Shutdown) {
    shutdown.exe /s /t 120 /c 'Unified21 V4 training completed. Run shutdown.exe /a within 120 seconds to cancel.'
}
