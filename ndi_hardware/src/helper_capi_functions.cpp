#include <unistd.h>

#include "helper_capi_functions.h"
#include "rclcpp/rclcpp.hpp"

void sleepSeconds(unsigned numSeconds)
{
#ifdef _WIN32
    Sleep((DWORD)1000 * numSeconds); // Sleep(ms)
#else
    sleep(numSeconds); // sleep(sec)
#endif
}

/**
 * @brief Prints a debug message if a method call failed.
 * @details To use, pass the method name and the error code returned by the method.
 *          Eg: onErrorPrintDebugMessage("capi.initialize()", capi.initialize());
 *          If the call succeeds, this method does nothing.
 *          If the call fails, this method prints an error message to stdout.
 */
void onErrorPrintDebugMessage(std::string methodName, int errorCode)
{
    std::stringstream res;
    if (errorCode < 0)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("NdiSensorHardwareInterface"), "NDI Hardware: " << methodName << " failed: " << capi.errorToString(errorCode));
    }
}


/**
 * @brief Returns the string: "[tool.id] s/n:[tool.serialNumber]" used in CSV output
 */
std::string getToolInfo(std::string toolHandle)
{
    // Get the port handle info from PHINF
    PortHandleInfo info = capi.portHandleInfo(toolHandle);

    // Return the ID and SerialNumber the desired string format
    std::string outputString = info.getToolId();
    outputString.append(" s/n:").append(info.getSerialNumber());
    return outputString;
}






/**
 * @brief Initialize and enable loaded tools. This is the same regardless of tool type.
 */
void initializeAndEnableTools()
{
    // Initialize and enable tools
    std::vector<PortHandleInfo> portHandles = capi.portHandleSearchRequest(PortHandleSearchRequestOption::NotInit);
    for (int i = 0; i < portHandles.size(); i++)
    {
        onErrorPrintDebugMessage("capi.portHandleInitialize()", capi.portHandleInitialize(portHandles[i].getPortHandle()));
        onErrorPrintDebugMessage("capi.portHandleEnable()", capi.portHandleEnable(portHandles[i].getPortHandle()));
    }

    // Print all enabled tools
    portHandles = capi.portHandleSearchRequest(PortHandleSearchRequestOption::Enabled);
}


/**
 * @brief Loads a tool from a tool definition file (.rom)
 */
std::string loadTool(const char *toolDefinitionFilePath)
{
    // Request a port handle to load a passive tool into
    int portHandle = capi.portHandleRequest();
    onErrorPrintDebugMessage("capi.portHandleRequest()", portHandle);

    // Load the .rom file using the previously obtained port handle
    capi.loadSromToPort(toolDefinitionFilePath, portHandle);
}


/**
 * @brief Demonstrate detecting active tools.
 * @details Active tools are connected through a System Control Unit (SCU) with physical wires.
 */
void configureActiveTools(std::string scuHostname)
{
    onErrorPrintDebugMessage("capi.setUserParameter()", capi.setUserParameter("Param.Connect.SCU Hostname", scuHostname));
}


/**
 * @brief Demonstrate loading dummy tools of each tool type.
 * @details Dummy tools are used to report 3Ds in the absence of real tools.
 *          Dummy tools should not be loaded with regular tools of the same type.
 *          TSTART will fail if real and dummy tools are enabled simultaneously.
 */
void configureDummyTools()
{
    onErrorPrintDebugMessage("capi.loadPassiveDummyTool()", capi.loadPassiveDummyTool());
    onErrorPrintDebugMessage("capi.loadActiveWirelessDummyTool()", capi.loadActiveWirelessDummyTool());
    onErrorPrintDebugMessage("capi.loadActiveDummyTool()", capi.loadActiveDummyTool());
}




/**
 * @brief Determines whether an NDI device supports the BX2 command by looking at the API revision
 */
void determineApiSupportForBX2()
{
    // Lookup the API revision
    std::string response = capi.getApiRevision();

    // Refer to the API guide for how to interpret the APIREV response
    char deviceFamily = response[0];
    int majorVersion = capi.stringToInt(response.substr(2, 3));

    // As of early 2017, the only NDI device supporting BX2 is the Vega
    // Vega is a Polaris device with API major version 003
    if (deviceFamily == 'G' && majorVersion >= 3)
    {
        apiSupportsBX2 = true;
    }
}
