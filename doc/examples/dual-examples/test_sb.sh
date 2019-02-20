# Run this test to reproduce the exeriments with Memorax
# ./test_sb.sh

#! /bin/bash


echo "Start doing experiments: SB model"

# Read tests cases from file
a=0
while read line
do
LINES[$a]=$line;
a=$(expr $a + 1);
done < "test_cases_sb.txt"


# Run tests
b=0
while [ $b -lt $a ]
do
${LINES[$b]};
b=$(expr $b + 1);
done

