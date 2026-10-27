; Inno Setup script for HADI OCR. Built by packaging/build.ps1, which passes
; the version: ISCC /DAppVer=1.6 installer.iss
;
; Each version installs side by side in its own folder, with its own shortcut
; and its own entry in Windows "Apps", so several versions can coexist.

#ifndef AppVer
  #error Pass the version: ISCC /DAppVer=1.6 installer.iss
#endif
#define AppExe "HADI_OCR_v" + AppVer + ".exe"

[Setup]
; Per-version AppId keeps versions independent (installing v1.7 leaves v1.6).
AppId=HADIOCR_v{#AppVer}
AppName=HADI OCR v{#AppVer}
AppVersion={#AppVer}
AppVerName=HADI OCR v{#AppVer}
; Per-user install: no admin prompt, and the app can save its settings
; (load cells, autosaves) next to itself.
PrivilegesRequired=lowest
DefaultDirName={localappdata}\Programs\HADI OCR\v{#AppVer}
DisableDirPage=yes
DisableProgramGroupPage=yes
DisableWelcomePage=no
OutputDir=..\dist\installer
OutputBaseFilename=HADI_OCR_Setup_v{#AppVer}
SetupIconFile=..\app_icon.ico
UninstallDisplayIcon={app}\{#AppExe}
UninstallDisplayName=HADI OCR v{#AppVer}
Compression=lzma2/max
SolidCompression=yes
WizardStyle=modern
CloseApplications=yes

[Tasks]
Name: "desktopicon"; Description: "Create a desktop shortcut"; GroupDescription: "Shortcuts:"

[Files]
Source: "..\dist\HADI_OCR_v{#AppVer}\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs

[Icons]
Name: "{autoprograms}\HADI OCR v{#AppVer}"; Filename: "{app}\{#AppExe}"
Name: "{autodesktop}\HADI OCR v{#AppVer}"; Filename: "{app}\{#AppExe}"; Tasks: desktopicon

[Run]
Filename: "{app}\{#AppExe}"; Description: "Start HADI OCR v{#AppVer}"; Flags: nowait postinstall skipifsilent

[UninstallDelete]
; Runtime files the app creates. Load cells and autosaves are left alone.
Type: files; Name: "{app}\freeze_log*.txt"
Type: files; Name: "{app}\sync_state.json"
Type: files; Name: "{app}\camera_prefs.json"
Type: files; Name: "{app}\decimal_point.json"

[Code]
{ Carry the load cells over from the most recently used other version, so a
  new version starts with them (only if this version has none yet). }
{ Last-write time of a file as two 32-bit halves; False if it can't be read. }
function GetWriteTime(const FileName: String; var High, Low: Cardinal): Boolean;
var
  F: TFindRec;
begin
  Result := FindFirst(FileName, F);
  if Result then
  begin
    High := F.LastWriteTime.dwHighDateTime;
    Low := F.LastWriteTime.dwLowDateTime;
    FindClose(F);
  end;
end;

procedure CopyLoadCellsFromPreviousVersion();
var
  Root, Target, Best, Candidate: String;
  Rec: TFindRec;
  BestHigh, BestLow, High, Low: Cardinal;
begin
  Target := ExpandConstant('{app}\load_cells.json');
  if FileExists(Target) then
    Exit;
  Root := ExpandConstant('{localappdata}\Programs\HADI OCR\');
  Best := '';
  BestHigh := 0;
  BestLow := 0;
  if FindFirst(Root + '*', Rec) then
  try
    repeat
      Candidate := Root + Rec.Name + '\load_cells.json';
      if ((Rec.Attributes and FILE_ATTRIBUTE_DIRECTORY) <> 0) and
         (Rec.Name <> '.') and (Rec.Name <> '..') and
         (CompareText(Root + Rec.Name, ExpandConstant('{app}')) <> 0) and
         GetWriteTime(Candidate, High, Low) then
      begin
        { the most recently saved load_cells.json wins }
        if (Best = '') or (High > BestHigh) or ((High = BestHigh) and (Low > BestLow)) then
        begin
          Best := Candidate;
          BestHigh := High;
          BestLow := Low;
        end;
      end;
    until not FindNext(Rec);
  finally
    FindClose(Rec);
  end;
  if Best <> '' then
    FileCopy(Best, Target, True);
end;

procedure CurStepChanged(CurStep: TSetupStep);
begin
  if CurStep = ssPostInstall then
    CopyLoadCellsFromPreviousVersion();
end;
