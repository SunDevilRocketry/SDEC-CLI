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

The CLI supports automatically connecting to a port specified in a file named `config.json` in the project root. For example, if your device usually connects over `COM4`, then your configuration would be `{"port":"COM4"}`. To tell the CLI to use your user config, start it with the `-c` or `--use_config` flags. For example, `python cli.py -c`.