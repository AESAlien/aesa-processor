param(
    [string]$PluginDirectory = (Join-Path $env:APPDATA 'Wireshark/plugins')
)

$ErrorActionPreference = 'Stop'
$sourcePath = Join-Path $PSScriptRoot 'aesa.lua'
$destinationPath = Join-Path $PluginDirectory 'aesa.lua'

New-Item -ItemType Directory -Path $PluginDirectory -Force | Out-Null
Copy-Item -LiteralPath $sourcePath -Destination $destinationPath -Force
Write-Output "Installed: $destinationPath"
Write-Output 'Restart Wireshark or select Analyze > Reload Lua Plugins.'
