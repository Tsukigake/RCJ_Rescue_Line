reg add "HKEY_CURRENT_USER\SOFTWARE\Microsoft\Windows\CurrentVersion\Internet Settings" /f /v ProxyEnable /t reg_dword /d 1
reg add "HKEY_CURRENT_USER\SOFTWARE\Microsoft\Windows\CurrentVersion\Internet Settings" /f /v ProxyServer /t reg_sz /d proxy01.kaiyo.ac.jp:8080
reg add "HKEY_CURRENT_USER\SOFTWARE\Microsoft\Windows\CurrentVersion\Internet Settings" /f /v ProxyOverride /t reg_sz /d "*.kaiyo.ac.jp"
