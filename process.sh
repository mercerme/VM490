#!/bin/bash

for file in ${1:-raw_data}/*.xml; do
	# echo "$file" "--output" "processed_data/$(basename "$file" .xml).csv"
	python xml2csv.py "$file" "--separator" "," "--output" "${2:-processed_data}/$(basename "$file" .xml).csv"
done

cp "${1:-raw_data}/params.json" "${2:-processed_data}/params.json"