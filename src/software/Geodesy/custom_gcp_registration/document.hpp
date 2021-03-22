#ifndef DOCUMENT_HPP
#define DOCUMENT_HPP

#pragma once

#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"

#include <fstream>
#include <cereal/archives/json.hpp>
#include <cereal/types/map.hpp>

#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

struct Document
{
    openMVG::sfm::SfM_Data _sfm_data;

    bool loadData(const std::string &sfm_data_ProjectPath)
    {
        return openMVG::sfm::Load(_sfm_data,
                                  sfm_data_ProjectPath,
                                  openMVG::sfm::ESfM_Data(openMVG::sfm::ALL));
    }

    bool saveData(const std::string &sFileName)
    {
        return openMVG::sfm::Save(_sfm_data,
                                  sFileName,
                                  openMVG::sfm::ESfM_Data(openMVG::sfm::EXTRINSICS));
    }
};

#endif /* DOCUMENT_HPP */