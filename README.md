A terminal application to provide commands for all SDECv2 functions. Compatible with SDEC v2.0.0.

Before running:
- pip install -e ./SDECv2
- add the following to your .vscode/settings.json to prevent errors on SDECv2 internal imports
```
{
  "python.analysis.extraPaths": [
    "${workspaceFolder}/SDECv2"
  ]
}
```