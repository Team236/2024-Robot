@echo off 
SETLOCAL EnableDelayedExpansion
set obj[0].Name="Mike DeFiore"
set obj[1].Name="Aaryan Sagar"
set obj[2].Name="Beth Viera"
set obj[3].Name="Elena Gerardo"
set obj[4].Name="Aaryan Sagar"
set obj[5].Name="Daniel Shaposhnikov"
set obj[6].Name="Steve Burnham"
set obj[7].Name="Ellen DiCarlo"
set obj[8].Name="Rob Reinhart"
set obj[9].Name="Steve Garabedian" 


FOR /L %%i IN (0 1 2 3 4 5 6 7 8 9) DO  (
   call echo %%i %%obj[%%i].Name%%
   set /a num = %%i
)
set /p choice=Type the number to select the coder from 0 to %num%: 
echo %choice%
IF %choice% GEQ 0 (
  IF %choice% LEQ 9 ( 
rem echo %obj[1].Name%
rem echo obj[%choice%].Name
rem echo !obj[%choice%].Name!
rem echo name %namevar%
echo !obj[%choice%].Name! has been added as the git user
git config --global user.name !obj[%choice%].Name!
  ) ELSE (  
    echo "Invalid selection"
	)
) ELSE (
 echo "Invalid selection"
)