#!/bn/bash

echo "Finding All Source Files"
echo "------------------------------------------"
find -name '*.cpp' -o -name '*.cc' -o -name '*.h' -o -name '*.c' > cscope.files


echo "Building CSCOPE Database"
echo "------------------------------------------"
cscope -R -b -q -k

echo "Setting path to CSCOPE Database"
echo "------------------------------------------"
export CSCOPE_DB=$PWD/cscope.out
