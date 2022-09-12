/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <memory>
#include <mutex>
#include <ostream>
#include <vector>
#include <string>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Subscriber.hh>
#include "gilbreth_gazebo/plugins/VacuumGripperPlugin.hh"
#include "gilbreth_gazebo/plugins/ARIAC.hh"
#include <ignition/math/Box.hh>

namespace gazebo
{
  /// \internal
  /// \brief Private data for the VacuumGripperPlugin class
  struct VacuumGripperPluginPrivate
  {
    /// \brief Class to store information about an object to be dropped.
    /// If the attached object is scheduled to be dropped, the drop will
    /// occur when the object enters inside the dropRegion. The object will
    /// be relocated to the respective pose.
    public: class DropObject
            {
              /// \brief Equality operator, result = this == _obj
              /// \param[in] _obj Object to check for equality
              /// \return true if this == _obj
              public: bool operator ==(const DropObject &_obj) const
              {
                return this->type == _obj.type && \
                  this->dropRegion == _obj.dropRegion && \
                  this->destination == _obj.destination;
              }

              /// \brief Stream insertion operator.
              /// \param[in] _out output stream
              /// \param[in] _obj object to output
              /// \return The output stream
              public: friend std::ostream &operator<<(std::ostream &_out,
                                                      const DropObject &_obj)
              {
                _out << _obj.type << std::endl;
                _out << _obj.dropRegion << std::endl;
                _out << "  Dst: [" << _obj.destination << "]" << std::endl;
                return _out;
              }

              /// \brief Object type.
              public: std::string type;

              /// \brief Object type.
              public: ignition::math::AxisAlignedBox dropRegion;

              /// \brief Destination where objects are teleported to after a drop
              public: ignition::math::Pose3d destination;

              /// \brief Getter for the type of object to drop
              public: std::string getType() const
              {
                return this->type;
              };
            };

    /// \brief Collection of objects that have been dropped.
    public: std::vector<std::string> droppedObjects;

    /// \brief Collection of objects to be dropped.
    public: std::vector<DropObject> objectsToDrop;

    /// \brief Model that contains this gripper.
    public: physics::ModelPtr model;

    /// \brief Pointer to the world.
    public: physics::WorldPtr world;

    /// \brief A fixed joint to connect the gripper to an object.
    public: physics::JointPtr fixedJoint;

    /// \brief The suction cup link.
    public: physics::LinkPtr suctionCupLink;

    /// \brief Connection event.
    public: event::ConnectionPtr connection;

    /// \brief The collisions for the links in the gripper.
    public: std::map<std::string, physics::CollisionPtr> collisions;

    /// \brief The current contacts.
    public: std::vector<msgs::Contact> contacts;

    /// \brief Mutex used to protect reading/writing the sonar message.
    public: std::mutex mutex;

    /// \brief True if the gripper has an object.
    public: bool attached = false;

    /// \brief The name of the attached link
    public: std::string attached_link_name = "";

    /// \brief Rate at which to update the gripper.
    public: common::Time updateRate;

    /// \brief Previous time when the gripper was updated.
    public: common::Time prevUpdateTime;

    /// \brief Number of iterations the gripper was contacting the same
    /// object.
    public: int posCount;

    /// \brief Number of iterations the gripper was not contacting the same
    /// object.
    public: int zeroCount;

    /// \brief Minimum number of links touching.
    public: unsigned int minContactCount;

    /// \brief Steps touching before engaging fixed joint
    public: int attachSteps;

    /// \brief Steps not touching before disengaging fixed joint
    public: int detachSteps;

    /// \brief Name of the gripper.
    public: std::string name;

    /// \brief Node for communication.
    public: transport::NodePtr node;

    /// \brief Subscription to contact messages from the physics engine.
    public: transport::SubscriberPtr contactSub;

    /// \brief Whether the suction is enabled or not.
    public: bool enabled = false;

    /// \brief Whether disabling of the suction has been requested or not.
    public: bool disableRequested = false;

    /// \brief Whether there's an ongoing drop.
    public: bool dropPending = false;

    /// \brief Attached model type.
    public: std::string attachedObjType;

    /// \brief Attached model to be dropped.
    public: physics::ModelPtr dropAttachedModel;

    /// \brief Collision with the model in contact.
    public: physics::CollisionPtr modelCollision;

    /// \brief Normal of the contact with the model in collision.
    public: ignition::math::Vector3d modelContactNormal;
  };
}

using namespace gazebo;
using namespace physics;

GZ_REGISTER_MODEL_PLUGIN(VacuumGripperPlugin)

/////////////////////////////////////////////////
VacuumGripperPlugin::VacuumGripperPlugin()
  : dataPtr(new VacuumGripperPluginPrivate)
{
  gzmsg << "VacuumGripper plugin loaded" << std::endl;

  this->dataPtr->attached = false;
  this->dataPtr->updateRate = common::Time(0, common::Time::SecToNano(0.1));
}

/////////////////////////////////////////////////
VacuumGripperPlugin::~VacuumGripperPlugin()
{
  if (this->dataPtr->world && this->dataPtr->world->Running())
  {
    auto mgr = this->dataPtr->world->Physics()->GetContactManager();
    mgr->RemoveFilter(this->Name());
  }
}

/////////////////////////////////////////////////
void VacuumGripperPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->dataPtr->model = _model;
  this->dataPtr->world = this->dataPtr->model->GetWorld();

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init(this->dataPtr->world->Name());
  this->dataPtr->name = _sdf->Get<std::string>("name");

  // Create the joint that will attach the objects to the suction cup
  this->dataPtr->fixedJoint =
      this->dataPtr->world->Physics()->CreateJoint(
        "fixed", this->dataPtr->model);
  this->dataPtr->fixedJoint->SetName(this->dataPtr->model->GetName() +
      "__vacuum_gripper_fixed_joint__");

  // Read the SDF parameters
  sdf::ElementPtr graspCheck = _sdf->GetElement("grasp_check");
  this->dataPtr->minContactCount =
      graspCheck->Get<unsigned int>("min_contact_count");
  this->dataPtr->attachSteps = graspCheck->Get<int>("attach_steps");
  this->dataPtr->detachSteps = graspCheck->Get<int>("detach_steps");
  sdf::ElementPtr suctionCupLinkElem = _sdf->GetElement("suction_cup_link");
  this->dataPtr->suctionCupLink =
    this->dataPtr->model->GetLink(suctionCupLinkElem->Get<std::string>());
  if (!this->dataPtr->suctionCupLink)
  {
    gzerr << "Suction cup link [" << suctionCupLinkElem->Get<std::string>()
          << "] not found!\n";
    return;
  }

  if (_sdf->HasElement("drops"))
  {
    sdf::ElementPtr dropsElem = _sdf->GetElement("drops");

    if (!dropsElem->HasElement("drop_regions"))
    {
      gzerr << "VacuumGripperPlugin: Unable to find <drop_regions> element in "
            << "the <drops> section\n";
      return;
    }

    sdf::ElementPtr dropRegionsElem = dropsElem->GetElement("drop_regions");
    sdf::ElementPtr dropRegionElem = NULL;
    if (dropRegionsElem->HasElement("drop_region"))
    {
      dropRegionElem = dropRegionsElem->GetElement("drop_region");
    }
    while (dropRegionElem)
    {
        if (!dropRegionElem->HasElement("min"))
      {
        gzerr << "VacuumGripperPlugin: Unable to find <min> elements in "
              << "the <drop_region> section\n";
        return;
      }

      sdf::ElementPtr minElem = dropRegionElem->GetElement("min");
      ignition::math::Vector3d min = dropRegionElem->Get<ignition::math::Vector3d>("min");

      if (!dropRegionElem->HasElement("max"))
      {
        gzerr << "VacuumGripperPlugin: Unable to find <max> elements in "
              << "the <drop_region> section\n";
        return;
      }

      sdf::ElementPtr maxElem = dropRegionElem->GetElement("max");
      ignition::math::Vector3d max = dropRegionElem->Get<ignition::math::Vector3d>("max");

      // Parse the destination.
      if (!dropRegionElem->HasElement("destination"))
      {
        gzerr << "VacuumGripperPlugin: Unable to find <destination> in "
              << "drop region\n";
        dropRegionElem = dropRegionElem->GetNextElement("drop_region");
        continue;
      }
      sdf::ElementPtr dstElement = dropRegionElem->GetElement("destination");

      // Parse the object type.
      if (!dropRegionElem->HasElement("type"))
      {
        gzerr << "VacuumGripperPlugin: Unable to find <type> in object.\n";
        dropRegionElem = dropRegionElem->GetNextElement("drop_region");
        continue;
      }
      sdf::ElementPtr typeElement = dropRegionElem->GetElement("type");
      std::string type = typeElement->Get<std::string>();

      ignition::math::AxisAlignedBox dropRegion = ignition::math::AxisAlignedBox(min, max);
      ignition::math::Pose3d destination = dstElement->Get<ignition::math::Pose3d>();
      VacuumGripperPluginPrivate::DropObject dropObject {type, dropRegion, destination};
      this->dataPtr->objectsToDrop.push_back(dropObject);

      dropRegionElem = dropRegionElem->GetNextElement("drop_region");
    }

  }
  else
  {
    gzdbg<<"Drop region for gripper not specified, disabling drop regions"<<std::endl;
  }

  // Find out the collision elements of the suction cup
  for (auto j = 0u; j < this->dataPtr->suctionCupLink->GetChildCount(); ++j)
  {
    physics::CollisionPtr collision =
       this->dataPtr->suctionCupLink->GetCollision(j);
    std::map<std::string, physics::CollisionPtr>::iterator collIter
      = this->dataPtr->collisions.find(collision->GetScopedName());
    if (collIter != this->dataPtr->collisions.end())
      continue;

    this->dataPtr->collisions[collision->GetScopedName()] = collision;
  }

  if (!this->dataPtr->collisions.empty())
  {
    // Create a filter to receive collision information
    auto mgr = this->dataPtr->world->Physics()->GetContactManager();
    auto topic = mgr->CreateFilter(this->Name(), this->dataPtr->collisions);
    if (!this->dataPtr->contactSub)
    {
      this->dataPtr->contactSub = this->dataPtr->node->Subscribe(topic,
          &VacuumGripperPlugin::OnContacts, this);
    }
  }

  this->Reset();

  this->dataPtr->connection = event::Events::ConnectWorldUpdateEnd(
      boost::bind(&VacuumGripperPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void VacuumGripperPlugin::Reset()
{
  this->dataPtr->prevUpdateTime = common::Time::GetWallTime();
  this->dataPtr->zeroCount = 0;
  this->dataPtr->posCount = 0;
  this->dataPtr->attached = false;
  this->dataPtr->enabled = false;
}

/////////////////////////////////////////////////
std::string VacuumGripperPlugin::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
bool VacuumGripperPlugin::Enabled() const
{
  return this->dataPtr->enabled;
}

/////////////////////////////////////////////////
bool VacuumGripperPlugin::Attached() const
{
  return this->dataPtr->attached;
}

/////////////////////////////////////////////////
void VacuumGripperPlugin::Enable()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->enabled = true;
}

/////////////////////////////////////////////////
void VacuumGripperPlugin::Disable()
{
  // Since we can't know what thread this gets called from, just set a flag
  // and the joint will be detached in the next OnUpdate callback in the physics thread.
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->disableRequested = true;
}

/////////////////////////////////////////////////
void VacuumGripperPlugin::OnUpdate()
{
  this->Publish();

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  if (this->dataPtr->disableRequested)
  {
    this->HandleDetach();
    this->dataPtr->enabled = false;
    this->dataPtr->disableRequested = false;
  }

  if(!this->dataPtr->enabled)
  {
    return;
  }

  if(common::Time::GetWallTime() -
      this->dataPtr->prevUpdateTime < this->dataPtr->updateRate )
  {
    return;
  }

  bool modelInContact = this->CheckModelContact();
  if (modelInContact)
  {
    this->HandleAttach();
  }

  if (this->dataPtr->attached && this->dataPtr->dropPending)
  {
    auto objPose = this->dataPtr->dropAttachedModel->WorldPose();
    for (const auto dropObject : this->dataPtr->objectsToDrop)
    {
      if (dropObject.type == this->dataPtr->attachedObjType && \
        dropObject.dropRegion.Contains(objPose.Pos()))
      {
        // Drop the object.
        this->HandleDetach();

        // Teleport it to the destination.
        this->dataPtr->dropAttachedModel->SetWorldPose(
          dropObject.destination);

        this->dataPtr->droppedObjects.push_back(this->dataPtr->attachedObjType);

        this->dataPtr->dropPending = false;
        gzdbg << "Object dropped and teleported" << std::endl;
        break;
      }
    }
  }

  // else if (this->dataPtr->zeroCount > this->dataPtr->detachSteps &&
  //          this->dataPtr->attached)
  // {
  //   this->HandleDetach();
  // }

  this->dataPtr->prevUpdateTime = common::Time::GetWallTime();
}

/////////////////////////////////////////////////
void VacuumGripperPlugin::OnContacts(ConstContactsPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->contacts.clear();
  for (int i = 0; i < _msg->contact_size(); ++i)
  {
    CollisionPtr collision1 = boost::dynamic_pointer_cast<Collision>(
        this->dataPtr->world->EntityByName(_msg->contact(i).collision1()));
    CollisionPtr collision2 = boost::dynamic_pointer_cast<Collision>(
        this->dataPtr->world->EntityByName(_msg->contact(i).collision2()));

    if ((collision1 && !collision1->IsStatic()) &&
        (collision2 && !collision2->IsStatic()))
    {
      this->dataPtr->contacts.push_back(_msg->contact(i));
    }
  }
}

/////////////////////////////////////////////////
bool VacuumGripperPlugin::GetContactNormal()
{
  physics::CollisionPtr collisionPtr;
  ignition::math::Vector3d contactNormal;
  this->dataPtr->modelCollision.reset();

  // Get the pointer to the collision that's not the gripper's.
  // This function is only called from the OnUpdate function so
  // the call to contacts.clear() is not going to happen in
  // parallel with the reads in the following code, no mutex needed.
  for (unsigned int i = 0; i < this->dataPtr->contacts.size(); ++i)
  {
    std::string name1 = this->dataPtr->contacts[i].collision1();
    std::string name2 = this->dataPtr->contacts[i].collision2();
    gzdbg << "Collision between '" << name1 << "' and '" << name2 << "'\n";

    if (this->dataPtr->collisions.find(name1) ==
        this->dataPtr->collisions.end())
    {
      // Model in contact is the second name
      this->dataPtr->modelCollision = boost::dynamic_pointer_cast<Collision>(
        this->dataPtr->world->EntityByName(name1));
      this->dataPtr->modelContactNormal = -1 * msgs::ConvertIgn(this->dataPtr->contacts[i].normal(0));
      return true;
    }

    if (this->dataPtr->collisions.find(name2) ==
        this->dataPtr->collisions.end())
    {
      // Model in contact is the first name -- frames are reversed
      this->dataPtr->modelCollision = boost::dynamic_pointer_cast<Collision>(
        this->dataPtr->world->EntityByName(name2));
      this->dataPtr->modelContactNormal = msgs::ConvertIgn(this->dataPtr->contacts[i].normal(0));
      return true;
    }
  }
/*
 * TODO: Delete, its unclear why this check even happens when no attempt to assign a value to it
 * is made
  if (!collisionPtr)
  {
    gzdbg << "Somehow the gripper was in collision with itself.\n";
  }*/

  return false;
}

/////////////////////////////////////////////////
void VacuumGripperPlugin::HandleAttach()
{
  if (this->dataPtr->attached)
  {
    return;
  }

  if(!this->dataPtr->modelCollision)
  {
    return;
  }



  this->dataPtr->fixedJoint->Load(this->dataPtr->suctionCupLink,
      this->dataPtr->modelCollision->GetLink(), ignition::math::Pose3d());
  this->dataPtr->fixedJoint->Init();

  this->dataPtr->attached_link_name = this->dataPtr->modelCollision->GetName();
  this->dataPtr->attached = true;
  gzdbg<<"Gripper attached object "<< this->dataPtr->attached_link_name <<std::endl;

  auto modelPtr = this->dataPtr->modelCollision->GetLink()->GetModel();
  auto name = modelPtr->GetName();
  gzdbg << "Part attached to gripper: " << name << std::endl;

  // Check if the object should drop.
  std::string objectType = ariac::DetermineModelType(name);
  auto it = find_if(this->dataPtr->objectsToDrop.begin(), this->dataPtr->objectsToDrop.end(),
    [&objectType](const VacuumGripperPluginPrivate::DropObject& obj) {
      return obj.getType() == objectType;
    });
  this->dataPtr->attachedObjType = objectType;
  bool objectToBeDropped = it != this->dataPtr->objectsToDrop.end();

  if (!objectToBeDropped)
  {
    return;
  }
  auto found = std::find(std::begin(this->dataPtr->droppedObjects),
                 std::end(this->dataPtr->droppedObjects), this->dataPtr->attachedObjType);
  bool alreadyDropped = found != std::end(this->dataPtr->droppedObjects);
  if (!alreadyDropped)
  {
    this->dataPtr->dropPending = true;
    this->dataPtr->dropAttachedModel = modelPtr;
    gzdbg << "Drop scheduled" << std::endl;
  }
}

/////////////////////////////////////////////////
void VacuumGripperPlugin::HandleDetach()
{
  gzdbg << "Detaching part from gripper." << std::endl;
  if(dataPtr->attached)
  {

    if(this->dataPtr->world->EntityByName(this->dataPtr->attached_link_name))
    {
      this->dataPtr->fixedJoint->Detach();
    }

    this->dataPtr->attached_link_name = "";
    this->dataPtr->modelCollision.reset();
    this->dataPtr->attached = false;
  }
}

/////////////////////////////////////////////////
bool VacuumGripperPlugin::CheckModelContact()
{
  bool modelInContact = false;
  if (this->dataPtr->contacts.size() > 0)
  {
    gzdbg << "Number of collisions with gripper: " << this->dataPtr->contacts.size() << std::endl;
  }
  if (this->dataPtr->contacts.size() >= this->dataPtr->minContactCount)
  {
    gzdbg << "More collisions than the minContactCount: " << this->dataPtr->minContactCount << std::endl;
    this->dataPtr->posCount++;
    this->dataPtr->zeroCount = 0;
  }
  else
  {
    this->dataPtr->zeroCount++;
    this->dataPtr->posCount = std::max(0, this->dataPtr->posCount-1);
  }

  if (this->dataPtr->posCount > this->dataPtr->attachSteps &&
      !this->dataPtr->attached)
  {
    if (!this->GetContactNormal())
    {
      return false;
    }
    // Only consider models with collision normals aligned with the normal of the gripper
    auto gripperLinkPose = this->dataPtr->suctionCupLink->WorldPose();
    ignition::math::Vector3d gripperLinkNormal =
      gripperLinkPose.Rot().RotateVector(ignition::math::Vector3d(0, -1, 0));
    double alignment = gripperLinkNormal.Dot(this->dataPtr->modelContactNormal);


    // Alignment of > 0.95 represents alignment angle of < acos(0.95) = ~18 degrees
    if (alignment > 0.95)
    {
      gzdbg << "Model contact normal: " << this->dataPtr->modelContactNormal.X() << " "
            << this->dataPtr->modelContactNormal.X() << " "
            << this->dataPtr->modelContactNormal.Z() << std::endl;

      gzdbg << "Dot Product: " << alignment << std::endl;
      gzdbg<<"Model is aligned with gripper"<<std::endl;
      modelInContact = true;
    }
  }
  return modelInContact;
}

/////////////////////////////////////////////////
void VacuumGripperPlugin::Publish() const
{
}
