@ECHO off
setlocal

SET current_dir=%~dp0
ECHO current_dir=%current_dir%
FOR %%a IN ("%current_dir:~0,-1%") DO SET root=%%~dpa
ECHO root=%root%

SUBST X: %root%

endlocal
@ECHO on
