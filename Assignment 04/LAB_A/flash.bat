@echo off
REM ------------------------------------------------------------------
REM flash.bat - Flash LAB_A.elf to the Nucleo-F446RE via STM32CubeProgrammer CLI.
REM
REM The STM32CubeIDE's ST-LINK GDB server cannot reliably spawn this CLI
REM helper from within its own process on this machine, so we invoke it
REM directly from a plain Windows command shell where it works correctly.
REM
REM Flags:
REM   -c port=SWD speed=fast mode=UR   Connect Under Reset (SWD, fast clock)
REM   -d "Debug/LAB_A.elf"            Download ELF to Flash
REM   -v                               Verify after programming
REM   -rst                             Reset target after programming
REM ------------------------------------------------------------------

setlocal
set "CUBEPROG=C:\ST\STM32CubeIDE_2.1.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.400.202601091506\tools\bin\STM32_Programmer_CLI.exe"
set "ELF=C:\CSE2-2\cse2206\Assignment 04\LAB_A\Debug\LAB_A.elf"

if not exist "%CUBEPROG%" (
    echo [flash.bat] ERROR: STM32_Programmer_CLI.exe not found at:
    echo   %CUBEPROG%
    exit /b 1
)

if not exist "%ELF%" (
    echo [flash.bat] ERROR: ELF not found at:
    echo   %ELF%
    echo Build the project first (Project -^> Build All in CubeIDE).
    exit /b 1
)

echo [flash.bat] Flashing: %ELF%
echo [flash.bat] Using   : %CUBEPROG%
echo.

"%CUBEPROG%" -c port=SWD speed=fast mode=UR -d "%ELF%" -v -rst

endlocal