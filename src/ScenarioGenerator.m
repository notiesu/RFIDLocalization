classdef ScenarioGenerator
    properties
        scenarioName(1,:) char
        readerState ReaderState
        tagStates(1,:) TagState
        rfidMeasurements(1,:) RFIDMeasurement
        controlCommands(1,:) ControlCommand
    end
end