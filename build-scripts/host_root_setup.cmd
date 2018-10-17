pip install paramiko
pip install requests
pip install tqdm
powershell -command "& { bash dl_game.bash $env:game_fileid}"
tar -xzf games.zip 
move games "%base_dir%\test_benches\"
