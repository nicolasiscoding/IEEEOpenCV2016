To make the c++ file, just cd into the directory where the file is and copy and paste this
to your terminal (without the arrows):




-------------->>>  g++ -o color_normalizer_exec color_normalizer.cpp `pkg-config opencv --cflags --libs`



This tells g++ where to find the despendencies for the file to compile.
It creates a file called color_normalizer_exec

After it compiles just type int your terminal (without the arrow):



-------------->>> ./color_normalizer_exec 