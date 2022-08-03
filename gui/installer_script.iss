; Script generated by the Inno Setup Script Wizard.
; SEE THE DOCUMENTATION FOR DETAILS ON CREATING INNO SETUP SCRIPT FILES!

#define MyAppName "UcoSLAMGUI"
#define MyAppVersion "1.1.0"
#define MyAppPublisher "Ava group"
#define MyAppURL "http://www.uco.es/grupos/ava/"
#define MyAppExeName "tslam_gui.exe"

[Setup]
; NOTE: The value of AppId uniquely identifies this application.
; Do not use the same AppId value in installers for other applications.
; (To generate a new GUID, click Tools | Generate GUID inside the IDE.)
AppId={{3100B24D-B81B-41C5-9945-29BD6FF36B40}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
AppPublisher={#MyAppPublisher}
AppPublisherURL={#MyAppURL}
AppSupportURL={#MyAppURL}
AppUpdatesURL={#MyAppURL}
DefaultDirName={pf}\{#MyAppName}
DefaultGroupName={#MyAppName}
OutputBaseFilename=UcoSLAMGUI-Installer-{#MyAppVersion}
SetupIconFile=/usr/local\bin\program_icon.ico
Compression=lzma
SolidCompression=yes
ChangesAssociations=yes


[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"
;Name: "spanish"; MessagesFile: "compiler:Languages\Spanish.isl"

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked

[Registry]

Root: HKCR; Subkey: ".map"; ValueType: string; ValueName: ""; ValueData: "EJE"; Flags: uninsdeletevalue
Root: HKCR; Subkey: "MAP"; ValueType: string; ValueName: ""; ValueData: "UcoSLAM Map File"; Flags: uninsdeletekey
Root: HKCR; Subkey: "MAP\DefaultIcon"; ValueType: string; ValueName: ""; ValueData: "{app}\program_icon.ico"
Root: HKCR; Subkey: "MAP\shell\open\command"; ValueType: string; ValueName: ""; ValueData: """{app}\{#MyAppExeName}"" ""%1"""

[Files]
Source: "/usr/local\bin\*"; DestDir: "{app}"; Flags: ignoreversion createallsubdirs recursesubdirs
Source: "vcredist_msvc2015_x64.exe"; DestDir: "{tmp}"
; NOTE: Don't use "Flags: ignoreversion" on any shared system files

[Icons]
Name: "{group}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}" ;   IconFilename: "{app}\program_icon.ico";
Name: "{commondesktop}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; Tasks: desktopicon ;IconFilename: "{app}\program_icon.ico";

[Run] 
Filename: "{tmp}\vcredist_msvc2015_x64.exe";
Filename: "{app}\{#MyAppExeName}"; Description: "{cm:LaunchProgram,{#StringChange(MyAppName, '&', '&&')}}"; Flags: nowait postinstall skipifsilent
