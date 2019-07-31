@Echo off
if not exist "..\libraries" mkdir ..\libraries
xcopy %~dp0\libraries\*.* ..\libraries\ /e