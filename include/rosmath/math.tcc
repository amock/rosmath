namespace rosmath {

template<typename GeomT, typename TupleEnabler<GeomT, TransformableTypes>::type* = nullptr>
std::vector<GeomT> mult(const geometry_msgs::Transform& T,
    const std::vector<GeomT>& data)
{
    std::vector<GeomT> ret(data.size());
    for(size_t i=0; i<data.size(); i++)
    {
        ret[i] = mult(T, data[i]);
    }
    return ret;
}

template<typename GeomT, typename TupleEnabler<GeomT, TransformableTypesStamped>::type* = nullptr>
std::vector<GeomT> mult(const geometry_msgs::TransformStamped& T,
    const std::vector<GeomT>& data)
{
    std::vector<GeomT> ret(data.size());
    for(size_t i=0; i<data.size(); i++)
    {
        ret[i] = mult(T, data[i]);
    }
    return ret;
}

template<typename GeomT, typename TupleEnabler<GeomT, RotatableTypes>::type* = nullptr>
std::vector<GeomT> mult(const geometry_msgs::Quaternion& q,
    const std::vector<GeomT>& data)
{
    std::vector<GeomT> ret(data.size());
    for(size_t i=0; i<data.size(); i++)
    {
        ret[i] = mult(q, data[i]);
    }
    return ret;
}

template<typename GeomT, typename TupleEnabler<GeomT, TransformableTypes>::type* = nullptr>
std::vector<GeomT> operator*(const geometry_msgs::Transform& T,
    const std::vector<GeomT>& data)
{
    return mult(T, data);
}

template<typename GeomT, typename TupleEnabler<GeomT, TransformableTypesStamped>::type* = nullptr>
std::vector<GeomT> operator*(const geometry_msgs::TransformStamped& T,
    const std::vector<GeomT>& data)
{
    return mult(T, data);
}

template<typename GeomT, typename TupleEnabler<GeomT, RotatableTypes>::type* = nullptr>
std::vector<GeomT> operator*(const geometry_msgs::Quaternion& q,
    const std::vector<GeomT>& data)
{
    return mult(q, data);
}


} // namespace rosmath