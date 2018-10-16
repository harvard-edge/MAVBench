pip install paramiko
pip install requests
pip install tqdm
powershell -command "& { bash dl_game.bash }"
tar -xzf games.zip 
move games "%base_dir%\test_benches\"
