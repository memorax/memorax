# Run this test to reproduce the exeriments with Memorax
# ./test_dual.sh

#! /bin/bash


echo "Start doing experiments: PDUAL model"

# Read tests cases from file
a=0
while read line
do
LINES[$a]=$line;
a=$(expr $a + 1);
done < "test_cases_pdual.txt"


# Run tests
b=0
while [ $b -lt $a ]
do
${LINES[$b]};
b=$(expr $b + 1);
done

