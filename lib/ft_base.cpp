#include "ft_base.hpp"

namespace ft_base
{
    FollowTargetBase::FollowTargetBase(FTBaseStruct tf_base_struct)
    {
        node_ptr_ = tf_base_struct.node_ptr;
        return;
    };
}