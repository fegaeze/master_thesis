
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
        key: "DROP_FOOT",
        title: "Drop Foot"
    }
]