/*******************************************************************************
 *BSD 3-Clause License
 *
 *Copyright (c) 2016-2022, Mech-Mind Robotics
 *All rights reserved.
 *
 *Redistribution and use in source and binary forms, with or without
 *modification, are permitted provided that the following conditions are met:
 *
 *1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 *2. Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 *3. Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 *THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include "SampleUtil.h"
#include "OpenCVUtil.h"
#include "PclUtil.h"

inline void getValue(double &value, std::string name)
{
    std::cout << "Please enter the " << name << ": ";
    std::cin >> value;
}

inline std::vector<std::vector<double>> allPossibleSubsets(std::vector<double> &values)
{
    int count = std::pow(2, values.size());
    std::vector<std::vector<double>> subsets;
    for (int i = 0; i < count; ++i)
    {
        std::vector<double> subset;
        for (int j = 0; j < values.size(); ++j)
        {
            if ((i & (1 << j)) != 0)
            {
                subset.emplace_back(values[j]);
            }
        }
        if (subset.size() != 0)
            subsets.emplace_back(subset);
    }
    return subsets;
}

inline std::string my_to_string(std::vector<double> &values)
{
    std::string str;
    for (int i = 0; i < values.size(); ++i)
    {
        str += std::to_string(values[i]);
        if (i != values.size() - 1)
            str += "_";
    }
    return str;
}

inline void capture3d(mmind::api::MechEyeDevice &device, std::string laserMode, std::vector<std::vector<double>> &_3dExposures, std::vector<std::pair<mmind::api::PointCloudProcessingSettings::CloudOutlierFilterMode, std::string>> &cloudOutlierFilterModes, std::vector<std::pair<mmind::api::PointCloudProcessingSettings::CloudSmoothMode, std::string>> &cloudSmoothModes)
{
    for (auto &_3dExp : _3dExposures)
    {
        showError(device.setScan3DExposure(_3dExp));
        for (auto &cloudOutlierFilterModePair : cloudOutlierFilterModes)
        {
            showError(device.setCloudOutlierFilterMode(cloudOutlierFilterModePair.first));
            for (auto &cloudSmoothModePair : cloudSmoothModes)
            {
                showError(device.setCloudSmoothMode(cloudSmoothModePair.first));
                std::string _3dSuffix = my_to_string(_3dExp) + "_Filter_" + cloudOutlierFilterModePair.second + "_Smooth_" + cloudSmoothModePair.second + laserMode;
                const std::string depthFile = _3dSuffix.empty() ? "DepthMap.png" : "DepthMap_" + _3dSuffix + ".tiff ";
                mmind::api::DepthMap depth;
                showError(device.captureDepthMap(depth));
                saveMap(depth, depthFile);
            }
        }
    }
}

inline void capture2d(mmind::api::MechEyeDevice &device, std::string _2dMode, std::string _2dExp)
{
    std::string _2dSuffix = _2dMode + _2dExp;
    const std::string colorFile = _2dSuffix.empty() ? "ColorMap.png" : "ColorMap_" + _2dSuffix + ".png ";
    mmind::api::ColorMap color;
    showError(device.captureColorMap(color));
    saveMap(color, colorFile);
}

inline void captureCombination(mmind::api::MechEyeDevice &device, std::string laserMode, double _2dLow, double _2dUp, double _2dStep, std::vector<std::vector<double>> &hdr2dExposures, std::vector<std::vector<double>> &_3dExposures, std::vector<std::pair<mmind::api::Scanning2DSettings::Scan2DExposureMode, std::string>> &_2dModes, std::vector<std::pair<mmind::api::PointCloudProcessingSettings::CloudOutlierFilterMode, std::string>> &cloudOutlierFilterModes, std::vector<std::pair<mmind::api::PointCloudProcessingSettings::CloudSmoothMode, std::string>> &cloudSmoothModes)
{
    for (auto &_2dModePair : _2dModes)
    {
        showError(device.setScan2DExposureMode(_2dModePair.first));
        if (_2dModePair.second == "Timed")
        {
            for (double _2dExp = _2dLow; _2dExp <= _2dUp; _2dExp += _2dStep)
            {
                showError(device.setScan2DExposureTime(_2dExp));
                capture2d(device, _2dModePair.second + "_", std::to_string(_2dExp));
            }
        }
        else if (_2dModePair.second == "HDR")
        {
            for (auto &hdr2dExposure : hdr2dExposures)
            {
                showError(device.setScan2DHDRExposureSequence(hdr2dExposure));
                capture2d(device, _2dModePair.second + "_", my_to_string(hdr2dExposure));
            }
        }
        else
        {
            capture2d(device, _2dModePair.second, "");
        }
    }
    capture3d(device, laserMode, _3dExposures, cloudOutlierFilterModes, cloudSmoothModes);
}

int main()
{
    mmind::api::MechEyeDevice device;
    if (!findAndConnect(device))
        return -1;

    mmind::api::MechEyeDeviceInfo deviceInfo;
    // deviceInfo.ipAddress = "127.0.0.1";
    // deviceInfo.port = 5577;
    // deviceInfo.firmwareVersion = "1.5.2";
    // showError(device.connect(deviceInfo));

    showError(device.getDeviceInfo(deviceInfo));
    printDeviceInfo(deviceInfo);

    double _2dLow, _2dUp, _2dStep, hdr2d1, hdr2d2, hdr2d3, hdr2d4, hdr2d5, _3dLow, _3dMid, _3dHi;
    getValue(_2dLow, "Lower bound for 2D exposure selection range in Timed mode");
    getValue(_2dUp, "Upper bound for 2D exposure selection range in Timed mode");
    getValue(_2dStep, "Step for 2D exposure selection in Timed mode");
    getValue(hdr2d1, "1st value for 2D exposure group in HDR mode");
    getValue(hdr2d2, "2nd value for 2D exposure group in HDR mode");
    getValue(hdr2d3, "3rd value for 2D exposure group in HDR mode");
    getValue(hdr2d4, "4th value for 2D exposure group in HDR mode");
    getValue(hdr2d5, "5th value for 2D exposure group in HDR mode");
    getValue(_3dLow, "1st value for 3D exposure group");
    getValue(_3dMid, "2nd value for 3D exposure group");
    getValue(_3dHi, "3rd value for 3D exposure group");

    std::vector<std::vector<double>> hdr2dExposures = allPossibleSubsets(std::vector<double>{hdr2d1, hdr2d2, hdr2d3, hdr2d4, hdr2d5});
    std::vector<std::vector<double>> _3dExposures = allPossibleSubsets(std::vector<double>{_3dLow, _3dMid, _3dHi});
    std::vector<std::pair<mmind::api::Scanning2DSettings::Scan2DExposureMode, std::string>> _2dModes{{mmind::api::Scanning2DSettings::Scan2DExposureMode::Timed, "Timed"}, {mmind::api::Scanning2DSettings::Scan2DExposureMode::Auto, "Auto"}, {mmind::api::Scanning2DSettings::Scan2DExposureMode::HDR, "HDR"}, {mmind::api::Scanning2DSettings::Scan2DExposureMode::Flash, "Flash"}};
    std::vector<std::pair<mmind::api::PointCloudProcessingSettings::CloudOutlierFilterMode, std::string>> cloudOutlierFilterModes{{mmind::api::PointCloudProcessingSettings::CloudOutlierFilterMode::Off, "Off"}, {mmind::api::PointCloudProcessingSettings::CloudOutlierFilterMode::Normal, "Normal"}, {mmind::api::PointCloudProcessingSettings::CloudOutlierFilterMode::Weak, "Weak"}};
    std::vector<std::pair<mmind::api::PointCloudProcessingSettings::CloudSmoothMode, std::string>> cloudSmoothModes{{mmind::api::PointCloudProcessingSettings::CloudSmoothMode::Off, "Off"}, {mmind::api::PointCloudProcessingSettings::CloudSmoothMode::Normal, "Normal"}, {mmind::api::PointCloudProcessingSettings::CloudSmoothMode::Weak, "Weak"}, {mmind::api::PointCloudProcessingSettings::CloudSmoothMode::Strong, "Strong"}};
    std::vector<std::pair<mmind::api::LaserSettings::LaserFringeCodingMode, std::string>> laserFringeCodingModes{{mmind::api::LaserSettings::LaserFringeCodingMode::Fast, "Fast"}, {mmind::api::LaserSettings::LaserFringeCodingMode::High, "High"}};

    if (deviceInfo.model.find("Laser") != std::string::npos)
    {
        mmind::api::LaserSettings laserSettings;
        showError(device.getLaserSettings(laserSettings));
        for (auto &laserFringeCodingModePair : laserFringeCodingModes)
        {
            laserSettings.FringeCodingMode = laserFringeCodingModePair.first;
            showError(device.setLaserSettings(laserSettings));
            captureCombination(device, "_Laser_" + laserFringeCodingModePair.second, _2dLow, _2dUp, _2dStep, hdr2dExposures, _3dExposures, _2dModes, cloudOutlierFilterModes, cloudSmoothModes);
        }
    }
    else
    {
        captureCombination(device, "", _2dLow, _2dUp, _2dStep, hdr2dExposures, _3dExposures, _2dModes, cloudOutlierFilterModes, cloudSmoothModes);
    }

    device.disconnect();
    std::cout << "Disconnected from the Mech-Eye device successfully." << std::endl;

    while (true)
    {
        std::string quitSignal;
        std::cout << "Enter 'quit' to quit: ";
        std::cin >> quitSignal;
        if (quitSignal == "quit")
            break;
    }
    return 0;
}
