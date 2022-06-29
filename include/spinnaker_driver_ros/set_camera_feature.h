#pragma once

// Spinnaker
#include "SpinGenApi/SpinnakerGenApi.h"
#include "Spinnaker.h"

/**
 * @brief Sets a camera feature

 * @param node_map Nodemap that includes the feature to set
 * @param feature_name Feature to be changed
 * @param feature_value Desired feature value
 * @returns True if succesful
 */
inline bool setFeature(Spinnaker::GenApi::INodeMap* node_map,
                       std::string feature_name, std::string feature_value) {
  Spinnaker::GenApi::CEnumerationPtr feature_ptr =
      node_map->GetNode(feature_name.c_str());

  if (!Spinnaker::GenApi::IsAvailable(feature_ptr)) {
    std::cerr << feature_name << " is not Available\n";
    return false;
  }

  if (!Spinnaker::GenApi::IsWritable(feature_ptr)) {
    std::cerr << feature_name << " is not Writable\n";
    return false;
  }

  Spinnaker::GenApi::CEnumEntryPtr entry_ptr =
      feature_ptr->GetEntryByName(feature_value.c_str());

  if (!Spinnaker::GenApi::IsAvailable(entry_ptr) ||
      !Spinnaker::GenApi::IsReadable(entry_ptr)) {
    std::cerr << feature_value << " for " << feature_name
              << " is not Available/Readable\n";
    return false;
  }

  const int64_t feature_value_int = entry_ptr->GetValue();
  feature_ptr->SetIntValue(feature_value_int);
  return true;
}

inline bool setFeature(Spinnaker::GenApi::INodeMap* node_map,
                       std::string feature_name, double feature_value) {
  Spinnaker::GenApi::CFloatPtr feature_ptr =
      node_map->GetNode(feature_name.c_str());

  if (!Spinnaker::GenApi::IsAvailable(feature_ptr)) {
    std::cerr << feature_name << " is not Available\n";
    return false;
  }

  if (!Spinnaker::GenApi::IsWritable(feature_ptr)) {
    std::cerr << feature_name << " is not Writable\n";
    return false;
  }

  // Check that value is in between bounds
  double actual_value = feature_value > feature_ptr->GetMax()
                            ? feature_ptr->GetMax()
                            : feature_value;
  actual_value = feature_value < feature_ptr->GetMin() ? feature_ptr->GetMin()
                                                       : feature_value;

  feature_ptr->SetValue(actual_value);
  return true;
}

inline bool setFeature(Spinnaker::GenApi::INodeMap* node_map,
                       std::string feature_name, int64_t feature_value) {
  Spinnaker::GenApi::CIntegerPtr feature_ptr =
      node_map->GetNode(feature_name.c_str());

  if (!Spinnaker::GenApi::IsAvailable(feature_ptr)) {
    std::cerr << feature_name << " is not Available\n";
    return false;
  }

  if (!Spinnaker::GenApi::IsWritable(feature_ptr)) {
    std::cerr << feature_name << " is not Writable\n";
    return false;
  }

  // Check that value is in between bounds
  int64_t actual_value = feature_value > feature_ptr->GetMax()
                             ? feature_ptr->GetMax()
                             : feature_value;
  actual_value = feature_value < feature_ptr->GetMin() ? feature_ptr->GetMin()
                                                       : feature_value;

  feature_ptr->SetValue(actual_value);
  return true;
}

inline bool setFeature(Spinnaker::GenApi::INodeMap* node_map,
                       std::string feature_name, bool feature_value) {
  Spinnaker::GenApi::CBooleanPtr feature_ptr =
      node_map->GetNode(feature_name.c_str());

  if (!Spinnaker::GenApi::IsReadable(feature_ptr)) {
    if (feature_ptr->GetValue() == feature_value) {
      std::cout << feature_name << " already set to desired value\n";
      return true;
    }
  }

  if (!Spinnaker::GenApi::IsAvailable(feature_ptr)) {
    std::cerr << feature_name << " is not Available\n";
    return false;
  }

  if (!Spinnaker::GenApi::IsWritable(feature_ptr)) {
    std::cerr << feature_name << " is not Writable\n";
    return false;
  }

  feature_ptr->SetValue(feature_value);
  return true;
}