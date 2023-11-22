
export const ROSBRIDGE_URL = `ws://localhost:8080`;
export const ACTION_SERVICE_URL = "/go1/action";
export const ACTION_SERVICE_TYPE = "go1_mud_test/ActionService";

export const CONTROLLER_TYPE_SERVICE_URL = "/go1/controller/type";
export const CONTROLLER_TYPE_SERVICE_TYPE = "go1_mud_test/ControllerTypeService";

export const PID_TUNING_SERVICE_URL = "/go1/controller/pid/gains";
export const PID_TUNING_SERVICE_TYPE = "go1_mud_test/PIDTuningService";

export const ACTION_SERVICE_ID = "ACTION";
export const CONTROLLER_TYPE_SERVICE_ID = "CONTROLLER_TYPE";
export const PID_TUNING_SERVICE_ID = "PID";


export const ctas = [
    {
        title: "Foot Controllers",
        id: CONTROLLER_TYPE_SERVICE_ID,
        actions: [
            {
                key: "PID",
                title: "PID"
            }, 
            { 
                key: "FIS",    
                title: "FIS"
            }
        ]
    },
    {
        title: "Robot Actions",
        id: ACTION_SERVICE_ID,
        actions: [
            {
                key: "STAND",
                title: "Stand"
            }, 
            { 
                key: "LIE_DOWN",    
                title: "Lie Down"
            },
            {   
                key: "RAISE_FR_FOOT",
                title: "Raise FR Foot"
            },
            {   
                key: "DROP_FOOT",
                title: "Drop Foot"
            }
        ]
    }
]