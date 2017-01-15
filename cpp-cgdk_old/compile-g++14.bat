set name=MyStrategy

if not exist %name%.cpp (
    echo Unable to find %name%.cpp > compilation.log
    exit 1
)

del /F /Q %name%.exe

set COMPILER_PATH=

if "%GCC6_HOME%" neq "" (
    if exist "%GCC6_HOME%\bin\g++.exe" (
        set COMPILER_PATH="%GCC6_HOME%\bin\"
    )
)

SetLocal EnableDelayedExpansion EnableExtensions

set FILES=

for %%i in (*.cpp) do (
    set FILES=!FILES! %%i
)

for %%i in (model\*.cpp) do (
    set FILES=!FILES! %%i
)

for %%i in (csimplesocket\*.cpp) do (
    set FILES=!FILES! %%i
)

"%COMPILER_PATH:"=%g++.exe" -std=c++14 -static -fno-optimize-sibling-calls -fno-strict-aliasing -DWIN32 -lm -s -x c++ -Wl,--stack=268435456 -O2 -Wall -Wtype-limits -Wno-unknown-pragmas -o %name%.exe!FILES! -lws2_32 -lwsock32 2>compilation.log
