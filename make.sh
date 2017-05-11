set -e

# Update sai2-common
cd sai2-common
git pull origin master
mkdir -p build_rel
cd build_rel
cmake ..
make -j4
cd ../..

# Build cs225a
mkdir -p build
cd build
cmake ..
make -j4
cd ..

# Download resource files
mkdir -p resources
cd resources
if [ ! -d "sawyer_graphics" ]; then
	curl -L http://cs.stanford.edu/groups/manips/teaching/cs225a/resources/sawyer_graphics.zip -o sawyer_graphics.zip
	unzip sawyer_graphics.zip
	rm sawyer_graphics.zip
fi
cd ..

cd bin
# Create controller files in bin
if [ -f "controller" ]; then
	cd resources/controller
	if [ ! -e "sawyer_graphics" ]; then
		ln -s ../../../resources/sawyer_graphics .
	fi
	cd ../..
fi
# Copy run_controller script from src
if [ ! -e "run_controller.sh" ]; then
	ln -s ../src/run_controller.sh .
fi
cd ..

cd bin
# Make script
cat <<EOF > make.sh
cd ..
mkdir -p build
cd build
cmake ..
make -j4
cd ../bin
EOF
chmod +x make.sh
cd ..
