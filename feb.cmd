@echo off
rem feb on Windows: the same launcher, run by the Python you installed (python.org or the Store)
where python >nul 2>nul && (python "%~dp0feb" %*) || (py -3 "%~dp0feb" %*)
