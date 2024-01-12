for DATAFILE in $(ls *.json)
do
    echo "Timetesting on file ${DATAFILE}"
    time python3 grid_map.py $DATAFILE
done

    
