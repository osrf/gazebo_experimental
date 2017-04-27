#include "gazebo/ecs/Componentizer.hh"

namespace gazebo
{
  namespace ecs
  {
    /// \brief Private implementation
    class ComponentizerPrivate
    {
    };

    Componentizer::Componentizer(Manager &_mgr) :
      dataPtr(new ComponentizerPrivate)
    {
    }

    ~Componentizer::Componentizer()
    {
    }

    void Componentizer::FromSDF(const sdf::SDf &_sdf)
    {
      // TODO for each element in sdf, ask  plugin if they want it
    }
  }
}
