
export const ROSBRIDGE_URL = `ws://localhost:8080`;
export const ACTION_SERVICE_URL = "/go1/action";
export const ACTION_SERVICE_TYPE = "go1_mud_test/ActionService";

export const actions = [
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
        key: "RAISE_FL_FOOT",
        title: "Raise FL Foot"
    },  
    {
        key: "RAISE_RR_FOOT",
        title: "Raise RR Foot"
    },  
    {
        key: "RAISE_RL_FOOT",
        title: "Raise RL Foot"
    }
]