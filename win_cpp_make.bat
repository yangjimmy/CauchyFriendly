@echo off
set FILENAME="cauchy_estimator"
set EXE="C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Tools\MSVC\14.43.34808\bin\Hostx64\x64\cl.exe"

set INC_MSVC=-I"C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Tools\MSVC\14.43.34808\include"
set INC_UCRT=-I"C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\ucrt"
set INC_WINUM=-I"C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um"
set INC_WINSHR=-I"C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\shared"

set LIB_PTHREAD=/LIBPATH:"D:\UCLA\Research\CauchyFilter\Code\CauchyFriendly\scripts\windows\pthread-win\Pre-built.2\lib\x64" pthreadVC2.lib
set LIB_UUID=/LIBPATH:"C:\Program Files (x86)\Windows Kits\10\Lib\10.0.22621.0\um\x64" uuid.lib
set LIB_CPMT=/LIBPATH:"C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Tools\MSVC\14.43.34808\lib\x64" libcpmt.lib
set LIB_UCRT=/LIBPATH:"C:\Program Files (x86)\Windows Kits\10\Lib\10.0.22621.0\ucrt\x64" libucrt.lib

echo "Removing old executable/object files from bin"
del /f ".\bin\%FILENAME%.exe"
del /f ".\bin\%FILENAME%.obj"
echo "Making %FILENAME%.exe located in bin folder..."
%EXE% /O2 /EHsc src/%FILENAME%.cpp %INC_MSVC% %INC_UCRT% %INC_WINUM% %INC_WINSHR% /Fo"bin/%FILENAME%.obj" /link %LIB_PTHREAD% %LIB_UUID% %LIB_CPMT% %LIB_UCRT% /OUT:bin/%FILENAME%.exe
