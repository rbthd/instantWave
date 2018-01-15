rm /home/robin/instantWave/minimu9/tests/results1_wave.csv 
rm /home/robin/instantWave/minimu9/tests/results1_wave_all.csv

touch /home/robin/instantWave/minimu9/tests/results1_wave.csv 
touch /home/robin/instantWave/minimu9/tests/results1_wave_all.csv 
cd /home/robin/instantWave/minimu9/minimu9-ahrs

while [ 1 ]
do
	echo -e `timeout --foreground 10 ./minimu9-ahrs -b /dev/i2c-1 --mode all` >> /home/robin/instantWave/minimu9/tests/results1_wave.csv
	echo "Suivant"
	echo -e `timeout --foreground 1 ./minimu9-ahrs -b /dev/i2c-1 --mode raw` >> /home/robin/instantWave/minimu9/tests/results1_wave_all.csv

done

