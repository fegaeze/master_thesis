import React from 'react'
import { Navigate, useLocation } from "react-router-dom"

import { useROS } from '../hooks/useROS';   

const ConnectedRoute = ({ children }) => {
    const { isConnected } = useROS();
    let location = useLocation();

    if(!isConnected) {
        return <Navigate to="/" state={{ from: location}} replace />
    }

    return children
};

export default ConnectedRoute;