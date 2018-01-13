#!/bin/bash

sudo rm /home/robin/minimu9/scripts/temp_results.txt
sudo touch /home/robin/minimu9/scripts/temp_results.txt
temp_results=/home/robin/minimu9/scripts/temp_results.txt
sudo chmod 777 $temp_results

## 200 first samples will be destroyed --> represents around 5s of running
echo "Start getting data from IMU"

# /!\ Be careful when changing the timeout since the 200 first samples will
# be destroyed (corresponding to 5 sec of running)
# remaining around 1500 samples

timeout 60 sudo ../minimu9-ahrs/minimu9-ahrs -b /dev/i2c-1 --mode all\
 | tee temp_results.txt #30s of real probing
echo "Data saved in temp file"

sudo cp temp_results.txt ./temp_results_back.txt #copy of original file before removing 200 first samples
												 #test/verification purpose

sudo sed -i '1,200d' $temp_results #actual removing
echo "200 first samples removed"

line=1
line_max=$(wc -l < $temp_results) #Renvoie le nombre de lignes du fichier
line_max=$[line_max+1]
i=0

northX=[]
northY=[]
northZ=[]
eastX=[]
eastY=[]
eastZ=[]
downX=[]
downY=[]
downZ=[]
quatA=[]
quatB=[]
quatC=[]
quatD=[]
yaw=[]
pitch=[]
roll=[]
vectAccX=[]
vectAccY=[]
vectAccZ=[]
vectMagX=[]
vectMagY=[]
vectMagZ=[]
MagMagnitude=[]

echo "Starting to save data in excel file"

while [ $line -lt $line_max ]
do

	## GET THE ORIENTATION DATA FROM LOG
	northX[i]=$(awk 'NR=='$line'{print $1}' $temp_results)
	#~ echo -e "northx = $northX"
	northY[i]=$(awk 'NR=='$line'{print $2}' $temp_results)
	#~ echo -e "northY = $northY"
	northZ[i]=$(awk 'NR=='$line'{print $3}' $temp_results)
	#~ echo -e "northZ = $northZ"
	
	eastX[i]=$(awk 'NR=='$line'{print $4}' $temp_results)
	#~ echo -e "eastX = $eastX"
	eastY[i]=$(awk 'NR=='$line'{print $5}' $temp_results)
	#~ echo -e "eastY = $eastY"
	eastZ[i]=$(awk 'NR=='$line'{print $6}' $temp_results)
	#~ echo -e "eastZ = $eastZ"
	
	downX[i]=$(awk 'NR=='$line'{print $7}' $temp_results)
	#~ echo -e "dozwX = $downX"
	downY[i]=$(awk 'NR=='$line'{print $8}' $temp_results)
	#~ echo -e "downY = $downY"
	downZ[i]=$(awk 'NR=='$line'{print $9}' $temp_results)
	#~ echo -e "downZ = $downZ"
	
	#QUATERNION
	quatA[i]=$(awk 'NR=='$line'{print $10}' $temp_results)
	#~ echo -e "quatA = $quatA"
	quatB[i]=$(awk 'NR=='$line'{print $11}' $temp_results)
	#~ echo -e "quatB = $quatB"
	quatC[i]=$(awk 'NR=='$line'{print $12}' $temp_results)
	#~ echo -e "quatC = $quatC"
	quatD[i]=$(awk 'NR=='$line'{print $13}' $temp_results)
	#~ echo -e "quatD = $quatD"
	
	#EULER ANGLES
	yaw[i]=$(awk 'NR=='$line'{print $14}' $temp_results)
	pitch[i]=$(awk 'NR=='$line'{print $15}' $temp_results)
	roll[i]=$(awk 'NR=='$line'{print $16}' $temp_results)

	#SCALED VECTORS
	vectAccX[i]=$(awk 'NR=='$line'{print $17}' $temp_results)  
	vectAccY[i]=$(awk 'NR=='$line'{print $18}' $temp_results)
	vectAccZ[i]=$(awk 'NR=='$line'{print $19}' $temp_results)
	
	vectMagX[i]=$(awk 'NR=='$line'{print $20}' $temp_results)
	vectMagY[i]=$(awk 'NR=='$line'{print $21}' $temp_results)
	vectMagZ[i]=$(awk 'NR=='$line'{print $22}' $temp_results)
	#~ echo -e "vectmagZ = $vectMagZ" 

	#~ echo -e " vect X = ${vectMagX[i]}"
	#~ echo -e " vect Y = ${vectMagY[i]}"
	#~ echo -e " vect Z = ${vectMagZ[i]}"
	
	MagMagnitude[i]=$(echo -e "scale=5;sqrt(${vectMagX[i]}*${vectMagX[i]} + ${vectMagY[i]}*${vectMagY[i]} + ${vectMagZ[i]}*${vectMagZ[i]})" | bc -l)
	
	#echo -e "magnitude= $MagMagnitude" 
			
	line=$[line+1]
	i=$[i+1]
	
done

	line_max=$[line_max-1]
	python3 /home/robin/minimu9/scripts/write-calc.py "${northX[*]}"\
	"${northY[*]}" "${northZ[*]}" "${eastX[*]}" "${eastY[*]}"\
	"${eastZ[*]}" "${downX[*]}" "${downY[*]}" "${downZ[*]}"\
	"${quatA[*]}" "${quatB[*]}" "${quatC[*]}" "${quatD[*]}"\
	"${yaw[*]}" "${pitch[*]}" "${roll[*]}"\
	"${vectAccX[*]}" "${vectAccY[*]}" "${vectAccZ[*]}"\
	"${vectMagX[*]}" "${vectMagY[*]}" "${vectMagZ[*]}" "$line_max" \
	"${MagMagnitude[*]}"
	
	## The saving currently takes 5 minutes which is too long
	## Will have to find another solution.
	## Xls file usefule for testing purposes such as plotting
	## XY graphs (magnetometer calibration)

echo "Data correctly fetched from IMU & saved in excel file"

unset northX[*]
unset northY[*]
unset northZ[*]
unset eastX[*]
unset eastY[*]
unset eastZ[*]
unset downX[*]
unset downY[*]
unset downZ[*]
unset quatA[*]
unset quatB[*]
unset quatC[*]
unset quatD[*]
unset yaw[*]
unset pitch[*]
unset roll[*]
unset vectAccX[*]
unset vectAccY[*]
unset vectAccZ[*]
unset vectMagX[*]
unset vectMagY[*]
unset vectMagZ[*]
unset MagMagnitude[*]

