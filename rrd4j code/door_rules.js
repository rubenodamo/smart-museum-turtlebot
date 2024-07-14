var entry_count = 0

rule "Entry Granted"

when

        Item Team_3_Door_Sensor_Sensor_Door changed from CLOSED to OPEN

then
        if (entry_count<100){
                // increments the entry count everytime the door state changes >
                entry_count = entry_count + 1
                var entryMessage = "Museum Entry Count: " + (entry_count).toStr>
                // sends updated message to openHab
                LogMessage.postUpdate(entryMessage)
                // send message to the log
                logInfo("DoorRule", entryMessage)
        }

        else {
                var capacityMessage = "Museum is FULL"
                LogMessage.postUpdate(capacityMessage)
                logInfo("DoorRule", capacityMessage)
        }
end


// resets the entry count to 0 when the button is pressed 5 times 
rule "Reset Entry Count"

when
        Item Team_3_Button_Scene_Number changed to 1.6

then
        entry_count = 0
        var resetMessage = "Museum Entry Count: " + (entry_count).toString
        LogMessage.postUpdate(resetMessage)
        logInfo("DoorRule", "Entry Count has been reset")

end

