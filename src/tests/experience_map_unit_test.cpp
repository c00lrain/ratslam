// ratslam
#include <ratslam/experience_map.h>
#include <utils/utils.h>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
// google test
#include <gtest/gtest.h>

using boost::property_tree::ptree;
using namespace ratslam;

TEST(ExperienceMap, testMissingConfigInitialization)
{
  // create in memory settings file
  std::stringstream ini_file_stream;
  ini_file_stream << "";

  // read settings file
  boost::property_tree::ptree settings, general_settings, ratslam_settings;
  read_ini(ini_file_stream, settings);

  // get settings
  std::string topic_root = "";
  EXPECT_FALSE(get_setting_child(ratslam_settings, settings, "ratslam", true));
  EXPECT_FALSE(get_setting_child(general_settings, settings, "general", true));
  get_setting_from_ptree(topic_root, general_settings, "topic_root", (std::string) "");

  // create the experience map object
  std::unique_ptr<ratslam::ExperienceMap> em(new ratslam::ExperienceMap(ratslam_settings));
}

std::unique_ptr<ratslam::ExperienceMap> createTestExperienceMap()
{
  // create in memory settings file
  std::stringstream ini_file_stream;
  ini_file_stream << ""
    "[general]\n"
    "topic_root=irat_red\n"
    "\n"
    "[ratslam]\n"
    "exp_delta_pc_threshold = 2\n"
    "exp_correction = 0.5\n"
    "exp_loops=20\n"
    "exp_initial_em_deg=140";

  // read settings file
  boost::property_tree::ptree settings, general_settings, ratslam_settings;
  read_ini(ini_file_stream, settings);
  // get settings
  std::string topic_root = "";
  EXPECT_TRUE(get_setting_child(ratslam_settings, settings, "ratslam", true));
  EXPECT_TRUE(get_setting_child(general_settings, settings, "general", true));
  get_setting_from_ptree(topic_root, general_settings, "topic_root", (std::string) "");
  EXPECT_TRUE(topic_root.compare("irat_red") == 0);

  // return new experience map object
  return std::unique_ptr<ratslam::ExperienceMap>(new ratslam::ExperienceMap(ratslam_settings));
}

TEST(ExperienceMap, testIniConfigInitalization)
{
  createTestExperienceMap();
}

bool load_map(const std::unique_ptr<ratslam::ExperienceMap>& em, const std::string& file_path)
{
  try
  {
    // create and open an archive for input
    std::ifstream ifs(file_path.c_str());
    boost::archive::binary_iarchive binary_archive(ifs);
    // read class state from archive
    binary_archive >> *em;
    // archive and stream closed when destructors are called
    return true;
  }
  catch (...)
  {
    return false;
  }
}

bool save_map(const std::unique_ptr<ratslam::ExperienceMap>& em, const std::string& file_path)
{
  try
  {
    // create and open a character archive for output
    std::ofstream ofs(file_path.c_str());

    // save map to archive file
    {
      boost::archive::binary_oarchive binary_archive(ofs);
      // write class instance to archive
      binary_archive << *em;
      // archive and stream closed when destructors are called
    }
    return true;
  }
  catch (...)
  {
    return false;
  }
}

TEST(ExperienceMap, testSaveLoad)
{
  // create the experience map object
  std::unique_ptr<ratslam::ExperienceMap> em = createTestExperienceMap();
  // store
  EXPECT_TRUE(save_map(em, "test.map"));
  // restore
  EXPECT_TRUE(load_map(em, "test.map"));
  // remove
  std::remove("test.map");
}

TEST(ExperienceMap, testCreateExperiences)
{
  // create the experience map object
  std::unique_ptr<ratslam::ExperienceMap> em = createTestExperienceMap();
  // add 10000 experiences
  for (int i = 0; i < 10000; ++i)
  {
    ASSERT_TRUE(em->on_create_experience(0) == i);
  }
}

TEST(ExperienceMap, testMapRelaxe)
{
  // create the experience map object
  std::unique_ptr<ratslam::ExperienceMap> em = createTestExperienceMap();

  // add 1000 experiences
  for (int i = 0; i < 1000; ++i)
  {
    ASSERT_TRUE(em->on_create_experience(0) == i);
  }

  // iterate 1000 times
  for (int i = 0; i < 1000; ++i)
  {
    ASSERT_TRUE(em->iterate());
  }
}

// Run all the tests
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}