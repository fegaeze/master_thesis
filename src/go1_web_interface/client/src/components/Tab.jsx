import React, { useState } from 'react';

const Tab = ({ children, headers }) => {
  const [activeTab, setActiveTab] = useState('tab1');

  const handleTabClick = (tabId) => {
    setActiveTab(tabId);
  };

  return (
    <div className="border rounded-lg">
      <ul className="flex bg-white rounded-t-lg">
        {
          headers.map((action, index) => {
            return (
              <li key={index}>
                <button
                  className={`px-6 py-3 focus:outline-none ${
                    activeTab === `tab${index + 1}`
                    ? 'bg-gray-100 text-indigo-500'
                    : 'bg-white text-gray-600 hover:bg-gray-100'
                  }`}
                  onClick={() => handleTabClick(`tab${index + 1}`)}
                >
                  {action}
                </button>
              </li>
            );
          })
        }
      </ul> 
      {
        children.map((child, index) => {
          return (
            <div key={index} className={`p-4 min-h-[300px] ${activeTab === `tab${index + 1}` ? '' : 'hidden'}`} id={`tab${index + 1}`}>
              {child}
            </div>
          );
        })
      }
    </div>
  );
};

export default Tab;
