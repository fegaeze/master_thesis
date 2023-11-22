
const CallToActionBtn = ({ action, title }) => {
    return (
        <button
            onClick={action}
            className={`relative px-6 py-2 bg-indigo-100 text-indigo-600 hover:bg-gradient-to-br from-indigo-500 to-indigo-700 hover:text-white focus:outline-none focus:ring focus:ring-indigo-300 rounded-md`}
        >
            {title}
        </button>
    );
}

export default CallToActionBtn;