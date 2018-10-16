fileid="1s9PORdVB8eS1LKyRdyiB6Qr9AP86IfqW" #blah
filename="games.zip" #blah
curl -c ./cookie -s -L "https://drive.google.com/uc?export=download&id=${fileid}" > /dev/null; #blah
curl -Lb ./cookie "https://drive.google.com/uc?export=download&confirm=`awk '/download/ {print $NF}' ./cookie`&id=${fileid}" -o games.zip #blah
# unzip games.zip do not unzip here. do it in windows otherwise it'll error out while running the executable

