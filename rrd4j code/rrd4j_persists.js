// How often we want the data of the item to be persisted 
Strategies{
    // Defined a strategy to persit data every minute  
    everyMinute : "0 * * * * ? "

}

// What item we want the data for 
Items {
    // Persists the amount of times the door has been opened 
     Team_3_Door_Sensor_Sensor_Door: strategy = everyChange

}