
const Card = ({ state, title }) => {
    return (
        <div className="p-6 border border-gray-200 bg-white rounded-lg text-center">
            <div className="text-2xl font-bold mt-2">{state || "N/A"}</div>
            <div className="text-xs mt-2 uppercase text-gray-500">{ title }</div>
        </div>
    );
}

export default Card;