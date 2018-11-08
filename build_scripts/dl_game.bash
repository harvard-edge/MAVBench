fileid=$1 #NEED THESE COMMENTS for windows  \r errors
#fileid="1s9PORdVB8eS1LKyRdyiB6Qr9AP86IfqW" NEED THESE COMMENTS for windows  \r errors
filename="games.zip" #NEED THESE COMMENTS for windows \r  errors
curl -c ./cookie -s -L "https://drive.google.com/uc?export=download&id=${fileid}" > /dev/null; #NEED THESE COMMENTS for windows \r errors
curl -Lb ./cookie "https://drive.google.com/uc?export=download&confirm=`awk '/download/ {print $NF}' ./cookie`&id=${fileid}" -o games.zip #NEED THESE COMMENTS for windows \r errors
# unzip games.zip do not unzip here. do it in windows otherwise it'll error out while running the executable
