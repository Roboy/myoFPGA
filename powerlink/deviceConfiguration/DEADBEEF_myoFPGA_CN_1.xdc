<?xml version="1.0" encoding="UTF-8"?>
<ISO15745ProfileContainer xmlns="http://www.ethernet-powerlink.org" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="http://www.ethernet-powerlink.org Powerlink_Main.xsd">
  <ISO15745Profile>
    <ProfileHeader>
      <ProfileIdentification>Powerlink_Device_Profile</ProfileIdentification>
      <ProfileRevision>1</ProfileRevision>
      <ProfileName>myoFPGA controlled node</ProfileName>
      <ProfileSource />
      <ProfileClassID>Device</ProfileClassID>
      <ISO15745Reference>
        <ISO15745Part>4</ISO15745Part>
        <ISO15745Edition>1</ISO15745Edition>
        <ProfileTechnology>Powerlink</ProfileTechnology>
      </ISO15745Reference>
    </ProfileHeader>
    <ProfileBody xsi:type="ProfileBody_Device_Powerlink" fileName="DEADBEEF_myoFPGA_CN.xdd" fileCreator="letrend" fileCreationDate="2017-03-24" fileCreationTime="12:00:00+01:00" fileModificationDate="2017-03-27" fileModificationTime="01:09:27+02:00" fileModifiedBy="roboy" fileVersion="01.00" supportedLanguages="en">
      <DeviceIdentity>
        <vendorName>roboy</vendorName>
        <vendorID>0xDEADBEEF</vendorID>
        <productName>myoFPGA</productName>
        <version versionType="HW">1.00</version>
        <version versionType="SW">1.00</version>
      </DeviceIdentity>
      <DeviceFunction>
        <capabilities>
          <characteristicsList>
            <characteristic>
              <characteristicName>
                <label lang="en">Transfer rate</label>
              </characteristicName>
              <characteristicContent>
                <label lang="en">100 MBit/s</label>
              </characteristicContent>
            </characteristic>
          </characteristicsList>
        </capabilities>
      </DeviceFunction>
    </ProfileBody>
  </ISO15745Profile>
  <ISO15745Profile>
    <ProfileHeader>
      <ProfileIdentification>Powerlink_Communication_Profile</ProfileIdentification>
      <ProfileRevision>1</ProfileRevision>
      <ProfileName>myoFPGA communication profile</ProfileName>
      <ProfileSource />
      <ProfileClassID>CommunicationNetwork</ProfileClassID>
      <ISO15745Reference>
        <ISO15745Part>4</ISO15745Part>
        <ISO15745Edition>1</ISO15745Edition>
        <ProfileTechnology>Powerlink</ProfileTechnology>
      </ISO15745Reference>
    </ProfileHeader>
    <ProfileBody xsi:type="ProfileBody_CommunicationNetwork_Powerlink" fileName="DEADBEEF_myoFPGA.xdd" fileCreator="letrend" fileCreationDate="2017-03-24" fileCreationTime="12:00:00+01:00" fileModificationDate="2017-03-27" fileModificationTime="01:09:27+02:00" fileModifiedBy="roboy" fileVersion="01.00" supportedLanguages="en">
      <ApplicationLayers>
        <identity>
          <vendorID>0xDEADBEEF</vendorID>
        </identity>
        <DataTypeList>
          <defType dataType="0001">
            <Boolean />
          </defType>
          <defType dataType="0002">
            <Integer8 />
          </defType>
          <defType dataType="0003">
            <Integer16 />
          </defType>
          <defType dataType="0004">
            <Integer32 />
          </defType>
          <defType dataType="0005">
            <Unsigned8 />
          </defType>
          <defType dataType="0006">
            <Unsigned16 />
          </defType>
          <defType dataType="0007">
            <Unsigned32 />
          </defType>
          <defType dataType="0008">
            <Real32 />
          </defType>
          <defType dataType="0009">
            <Visible_String />
          </defType>
          <defType dataType="0010">
            <Integer24 />
          </defType>
          <defType dataType="0011">
            <Real64 />
          </defType>
          <defType dataType="0012">
            <Integer40 />
          </defType>
          <defType dataType="0013">
            <Integer48 />
          </defType>
          <defType dataType="0014">
            <Integer56 />
          </defType>
          <defType dataType="0015">
            <Integer64 />
          </defType>
          <defType dataType="000A">
            <Octet_String />
          </defType>
          <defType dataType="000B">
            <Unicode_String />
          </defType>
          <defType dataType="000C">
            <Time_of_Day />
          </defType>
          <defType dataType="000D">
            <Time_Diff />
          </defType>
          <defType dataType="000F">
            <Domain />
          </defType>
          <defType dataType="0016">
            <Unsigned24 />
          </defType>
          <defType dataType="0018">
            <Unsigned40 />
          </defType>
          <defType dataType="0019">
            <Unsigned48 />
          </defType>
          <defType dataType="001A">
            <Unsigned56 />
          </defType>
          <defType dataType="001B">
            <Unsigned64 />
          </defType>
          <defType dataType="0401">
            <MAC_ADDRESS />
          </defType>
          <defType dataType="0402">
            <IP_ADDRESS />
          </defType>
          <defType dataType="0403">
            <NETTIME />
          </defType>
        </DataTypeList>
        <ObjectList>
          <!-- Communication Profile Area (0x1000 - 0x1FFF): defined by EPSG 301 -->
          <Object index="1000" name="NMT_DeviceType_U32" objectType="7" dataType="0007" accessType="const" PDOmapping="no" defaultValue="0x00000000" />
          <Object index="1001" name="ERR_ErrorRegister_U8" objectType="7" dataType="0005" accessType="ro" PDOmapping="optional" defaultValue="0" />
          <Object index="1006" name="NMT_CycleLen_U32" objectType="7" dataType="0007" accessType="rw" PDOmapping="no" defaultValue="0" actualValue="0x0000C350" />
          <Object index="1018" name="NMT_IdentityObject_REC" objectType="9">
            <SubObject subIndex="00" name="NumberOfEntries" objectType="7" dataType="0005" accessType="const" PDOmapping="no" defaultValue="4" />
            <SubObject subIndex="01" name="VendorId_U32" objectType="7" dataType="0007" accessType="const" PDOmapping="no" defaultValue="0x00000000" />
            <SubObject subIndex="02" name="ProductCode_U32" objectType="7" dataType="0007" accessType="const" PDOmapping="no" defaultValue="0x00000000" />
            <SubObject subIndex="03" name="RevisionNo_U32" objectType="7" dataType="0007" accessType="const" PDOmapping="no" defaultValue="0x00000000" />
            <SubObject subIndex="04" name="SerialNo_U32" objectType="7" dataType="0007" accessType="const" PDOmapping="no" defaultValue="0x00000000" />
          </Object>
          <Object index="1020" name="CFM_VerifyConfiguration_REC" objectType="9">
            <SubObject subIndex="00" name="NumberOfEntries" objectType="7" dataType="0005" accessType="const" defaultValue="2" />
            <SubObject subIndex="01" name="ConfDate_U32" objectType="7" dataType="0007" accessType="rw" defaultValue="0" actualValue="0x00002F6B" />
            <SubObject subIndex="02" name="ConfTime_U32" objectType="7" dataType="0007" accessType="rw" defaultValue="0" actualValue="0x003F9347" />
          </Object>
          <Object index="1030" name="NMT_InterfaceGroup_0h_REC" objectType="9">
            <SubObject subIndex="00" name="NumberOfEntries" objectType="7" dataType="0005" accessType="const" PDOmapping="no" defaultValue="9" />
            <SubObject subIndex="01" name="InterfaceIndex_U16" objectType="7" dataType="0006" accessType="ro" PDOmapping="no" defaultValue="1" />
            <SubObject subIndex="02" name="InterfaceDescription_VSTR" objectType="7" dataType="0009" accessType="const" PDOmapping="no" defaultValue="Interface 1" />
            <SubObject subIndex="03" name="InterfaceType_U8" objectType="7" dataType="0005" accessType="const" PDOmapping="no" defaultValue="6" />
            <SubObject subIndex="04" name="InterfaceMtu_U16" objectType="7" dataType="0006" accessType="const" PDOmapping="no" defaultValue="1500" />
            <SubObject subIndex="05" name="InterfacePhysAddress_OSTR" objectType="7" dataType="000A" accessType="const" PDOmapping="no" />
            <SubObject subIndex="06" name="InterfaceName_VSTR" objectType="7" dataType="0009" accessType="ro" PDOmapping="no" defaultValue="Interface 1" />
            <SubObject subIndex="07" name="InterfaceOperStatus_U8" objectType="7" dataType="0005" accessType="ro" PDOmapping="no" defaultValue="1" />
            <SubObject subIndex="08" name="InterfaceAdminState_U8" objectType="7" dataType="0005" accessType="rw" PDOmapping="no" defaultValue="1" />
            <SubObject subIndex="09" name="Valid_BOOL" objectType="7" dataType="0001" accessType="rw" PDOmapping="no" defaultValue="true" />
          </Object>
          <Object index="1300" name="SDO_SequLayerTimeout_U32" objectType="7" PDOmapping="no" accessType="rw" dataType="0007" defaultValue="5000" />
          <Object index="1400" name="PDO_RxCommParam_00h_REC" objectType="9">
            <SubObject subIndex="00" name="NumberOfEntries" objectType="7" dataType="0005" accessType="const" PDOmapping="no" defaultValue="2" />
            <SubObject subIndex="01" name="NodeID_U8" objectType="7" dataType="0005" accessType="rw" PDOmapping="no" defaultValue="0" actualValue="0xF0" />
            <SubObject subIndex="02" name="MappingVersion_U8" objectType="7" dataType="0005" accessType="rw" PDOmapping="no" defaultValue="0" />
          </Object>
          <Object index="1600" name="PDO_RxMappParam_00h_AU64" objectType="8">
            <SubObject subIndex="00" name="NumberOfEntries" objectType="7" dataType="0005" accessType="rw" PDOmapping="no" defaultValue="0" actualValue="0x0B" />
            <SubObject subIndex="01" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" actualValue="0x0020000000016000" />
            <SubObject subIndex="02" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" actualValue="0x0020002000026000" />
            <SubObject subIndex="03" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" actualValue="0x0020004000036000" />
            <SubObject subIndex="04" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" actualValue="0x0020006000046000" />
            <SubObject subIndex="05" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" actualValue="0x0010008000056000" />
            <SubObject subIndex="06" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" actualValue="0x0010009000066000" />
            <SubObject subIndex="07" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" actualValue="0x001000A000076000" />
            <SubObject subIndex="08" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" actualValue="0x001000B000086000" />
            <SubObject subIndex="09" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" actualValue="0x001000C000096000" />
            <SubObject subIndex="0A" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" actualValue="0x001000D0000A6000" />
            <SubObject subIndex="0B" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" actualValue="0x001000E0000B6000" />
            <SubObject subIndex="0C" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" />
            <SubObject subIndex="0D" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" />
            <SubObject subIndex="0E" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" />
            <SubObject subIndex="0F" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" />
            <SubObject subIndex="10" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" />
            <SubObject subIndex="11" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" />
            <SubObject subIndex="12" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" />
            <SubObject subIndex="13" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" />
            <SubObject subIndex="14" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" />
            <SubObject subIndex="15" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" />
            <SubObject subIndex="16" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" />
            <SubObject subIndex="17" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" />
            <SubObject subIndex="18" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" />
            <SubObject subIndex="19" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" />
          </Object>
          <Object index="1800" name="PDO_TxCommParam_00h_REC" objectType="9">
            <SubObject subIndex="00" name="NumberOfEntries" objectType="7" dataType="0005" accessType="const" PDOmapping="no" defaultValue="2" />
            <SubObject subIndex="01" name="NodeID_U8" objectType="7" dataType="0005" accessType="rw" PDOmapping="no" defaultValue="0" />
            <SubObject subIndex="02" name="MappingVersion_U8" objectType="7" dataType="0005" accessType="rw" PDOmapping="no" defaultValue="0" />
          </Object>
          <Object index="1A00" name="PDO_TxMappParam_00h_AU64" objectType="8">
            <SubObject subIndex="00" name="NumberOfEntries" objectType="7" dataType="0005" accessType="rw" PDOmapping="no" defaultValue="0" actualValue="0x07" />
            <SubObject subIndex="01" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" actualValue="0x0010000000016001" />
            <SubObject subIndex="02" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" actualValue="0x0020001000026001" />
            <SubObject subIndex="03" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" actualValue="0x0010003000036001" />
            <SubObject subIndex="04" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" actualValue="0x0010004000046001" />
            <SubObject subIndex="05" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" actualValue="0x0010005000056001" />
            <SubObject subIndex="06" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" actualValue="0x0010006000066001" />
            <SubObject subIndex="07" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" actualValue="0x0010007000076001" />
            <SubObject subIndex="08" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" />
            <SubObject subIndex="09" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" />
            <SubObject subIndex="0A" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" />
            <SubObject subIndex="0B" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" />
            <SubObject subIndex="0C" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" />
            <SubObject subIndex="0D" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" />
            <SubObject subIndex="0E" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" />
            <SubObject subIndex="0F" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" />
            <SubObject subIndex="10" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" />
            <SubObject subIndex="11" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" />
            <SubObject subIndex="12" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" />
            <SubObject subIndex="13" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" />
            <SubObject subIndex="14" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" />
            <SubObject subIndex="15" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" />
            <SubObject subIndex="16" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" />
            <SubObject subIndex="17" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" />
            <SubObject subIndex="18" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" />
            <SubObject subIndex="19" name="ObjectMapping" objectType="7" dataType="001B" accessType="rw" PDOmapping="no" defaultValue="0x0000000000000000" />
          </Object>
          <Object index="1C0B" name="DLL_CNLossSoC_REC" objectType="9">
            <SubObject subIndex="00" name="NumberOfEntries" objectType="7" dataType="0005" accessType="const" PDOmapping="no" defaultValue="3" />
            <SubObject subIndex="01" name="CumulativeCnt_U32" objectType="7" dataType="0007" accessType="rw" PDOmapping="no" />
            <SubObject subIndex="02" name="ThresholdCnt_U32" objectType="7" dataType="0007" accessType="ro" PDOmapping="no" />
            <SubObject subIndex="03" name="Threshold_U32" objectType="7" dataType="0007" accessType="rw" PDOmapping="no" defaultValue="15" actualValue="0x00000050" />
          </Object>
          <Object index="1C0F" name="DLL_CNCRCError_REC" objectType="9">
            <SubObject subIndex="00" name="NumberOfEntries" objectType="7" dataType="0005" accessType="const" PDOmapping="no" defaultValue="3" />
            <SubObject subIndex="01" name="CumulativeCnt_U32" objectType="7" dataType="0007" accessType="rw" PDOmapping="no" />
            <SubObject subIndex="02" name="ThresholdCnt_U32" objectType="7" dataType="0007" accessType="ro" PDOmapping="no" />
            <SubObject subIndex="03" name="Threshold_U32" objectType="7" dataType="0007" accessType="rw" PDOmapping="no" defaultValue="15" />
          </Object>
          <Object index="1C14" name="DLL_CNLossOfSocTolerance_U32" objectType="7" dataType="0007" accessType="rw" defaultValue="300000" actualValue="0x02FAF080" />
          <Object index="1F82" name="NMT_FeatureFlags_U32" objectType="7" dataType="0007" accessType="const" PDOmapping="no" defaultValue="0x00000045" />
          <Object index="1F83" name="NMT_EPLVersion_U8" objectType="7" dataType="0005" accessType="const" PDOmapping="no" defaultValue="0x20" />
          <Object index="1F8C" name="NMT_CurrNMTState_U8" objectType="7" dataType="0005" PDOmapping="optional" accessType="ro" />
          <Object index="1F93" name="NMT_EPLNodeID_REC" objectType="9">
            <SubObject subIndex="00" name="NumberOfEntries" objectType="7" dataType="0005" accessType="const" PDOmapping="no" defaultValue="2" />
            <SubObject subIndex="01" name="NodeID_U8" objectType="7" dataType="0005" accessType="ro" PDOmapping="no" />
            <SubObject subIndex="02" name="NodeIDByHW_BOOL" objectType="7" dataType="0001" accessType="ro" PDOmapping="no" defaultValue="true" />
          </Object>
          <Object index="1F98" name="NMT_CycleTiming_REC" objectType="9">
            <SubObject subIndex="00" name="NumberOfEntries" objectType="7" dataType="0005" accessType="const" PDOmapping="no" defaultValue="8" />
            <SubObject subIndex="01" name="IsochrTxMaxPayload_U16" objectType="7" dataType="0006" accessType="const" PDOmapping="no" defaultValue="1490" />
            <SubObject subIndex="02" name="IsochrRxMaxPayload_U16" objectType="7" dataType="0006" accessType="const" PDOmapping="no" defaultValue="1490" />
            <SubObject subIndex="03" name="PResMaxLatency_U32" objectType="7" dataType="0007" accessType="const" PDOmapping="no" defaultValue="2000" />
            <SubObject subIndex="04" name="PReqActPayloadLimit_U16" objectType="7" dataType="0006" accessType="rw" PDOmapping="no" defaultValue="36" />
            <SubObject subIndex="05" name="PResActPayloadLimit_U16" objectType="7" dataType="0006" accessType="rw" PDOmapping="no" defaultValue="36" actualValue="0x0024" />
            <SubObject subIndex="06" name="ASndMaxLatency_U32" objectType="7" dataType="0007" accessType="const" PDOmapping="no" defaultValue="2000" />
            <SubObject subIndex="07" name="MultiplCycleCnt_U8" objectType="7" dataType="0005" accessType="rw" PDOmapping="no" defaultValue="0" actualValue="0x00" />
            <SubObject subIndex="08" name="AsyncMTU_U16" objectType="7" dataType="0006" accessType="rw" PDOmapping="no" defaultValue="300" actualValue="0x012C" />
          </Object>
          <Object index="1F99" name="NMT_CNBasicEthernetTimeout_U32" objectType="7" dataType="0007" accessType="rw" PDOmapping="no" defaultValue="5000000" />
          <Object index="1F9E" name="NMT_ResetCmd_U8" objectType="7" dataType="0005" accessType="rw" PDOmapping="no" defaultValue="255" />
          <!-- Manufacturer Specific Profile Area (0x2000 - 0x5FFF): may freely be used by the device manufacturer -->
          <!-- Standardised Device Profile Area (0x6000 - 0x9FFF): may be used according to a CiA device profile. The profile to be used is given by NMT_DeviceType_U32 -->
          <Object index="6000" name="PID_controller_config_REC" objectType="9">
            <SubObject subIndex="00" name="NumberOfEntries" objectType="7" dataType="0005" accessType="const" PDOmapping="no" defaultValue="11" />
            <SubObject subIndex="01" name="outputPosMax_I32" objectType="7" dataType="0004" accessType="rw" PDOmapping="RPDO" defaultValue="1000" />
            <SubObject subIndex="02" name="outputNegMax_I32" objectType="7" dataType="0004" accessType="rw" PDOmapping="RPDO" defaultValue="-1000" />
            <SubObject subIndex="03" name="spPosMax_I32" objectType="7" dataType="0004" accessType="rw" PDOmapping="RPDO" defaultValue="1000000" />
            <SubObject subIndex="04" name="spNegMax_I32" objectType="7" dataType="0004" accessType="rw" PDOmapping="RPDO" defaultValue="-1000000" />
            <SubObject subIndex="05" name="Kp_U16" objectType="7" dataType="0006" accessType="rw" PDOmapping="RPDO" defaultValue="1" />
            <SubObject subIndex="06" name="Ki_U16" objectType="7" dataType="0006" accessType="rw" PDOmapping="RPDO" defaultValue="0" />
            <SubObject subIndex="07" name="Kd_U16" objectType="7" dataType="0006" accessType="rw" PDOmapping="RPDO" defaultValue="0" />
            <SubObject subIndex="08" name="forwardGain_U16" objectType="7" dataType="0006" accessType="rw" PDOmapping="RPDO" defaultValue="0" />
            <SubObject subIndex="09" name="deadBand_U16" objectType="7" dataType="0006" accessType="rw" PDOmapping="RPDO" defaultValue="0" />
            <SubObject subIndex="0A" name="IntegralPosMax_I16" objectType="7" dataType="0003" accessType="rw" PDOmapping="RPDO" defaultValue="0" />
            <SubObject subIndex="0B" name="IntegralNegMax_I16" objectType="7" dataType="0003" accessType="rw" PDOmapping="RPDO" defaultValue="0" />
          </Object>
          <Object index="6001" name="motorStatus_REC" objectType="9">
            <SubObject subIndex="00" name="NumberOfEntries" objectType="7" dataType="0005" accessType="const" PDOmapping="no" defaultValue="7" />
            <SubObject subIndex="01" name="pwmRef_I16" objectType="7" dataType="0006" accessType="ro" PDOmapping="TPDO" />
            <SubObject subIndex="02" name="actualPosition_I32" objectType="7" dataType="0004" accessType="ro" PDOmapping="TPDO" />
            <SubObject subIndex="03" name="actualVelocity_I16" objectType="7" dataType="0003" accessType="ro" PDOmapping="TPDO" />
            <SubObject subIndex="04" name="actualCurrent_I16" objectType="7" dataType="0003" accessType="ro" PDOmapping="TPDO" />
            <SubObject subIndex="05" name="springDisplacement_I16" objectType="7" dataType="0003" accessType="ro" PDOmapping="TPDO" />
            <SubObject subIndex="06" name="sensor1_I16" objectType="7" dataType="0003" accessType="ro" PDOmapping="TPDO" />
            <SubObject subIndex="07" name="sensor2_I16" objectType="7" dataType="0003" accessType="ro" PDOmapping="TPDO" />
          </Object>
        </ObjectList>
      </ApplicationLayers>
      <TransportLayers />
      <NetworkManagement>
        <GeneralFeatures DLLFeatureMN="false" NMTBootTimeNotActive="3000000" NMTCycleTimeMin="0" NMTCycleTimeMax="4294967295" NMTErrorEntries="2" NWLIPSupport="false" SDOServer="true" SDOMaxConnections="2" SDOMaxParallelConnections="2" SDOCmdWriteAllByIndex="false" SDOCmdReadAllByIndex="false" SDOCmdWriteByName="false" SDOCmdReadByName="false" SDOCmdWriteMultParam="false" NMTFlushArpEntry="false" NMTNetHostNameSet="false" PDORPDOChannels="1" PDORPDOChannelObjects="25" PDOSelfReceipt="false" PDOTPDOChannelObjects="25" />
        <CNFeatures DLLCNFeatureMultiplex="false" DLLCNPResChaining="false" NMTCNSoC2PReq="0" />
        <Diagnostic />
      </NetworkManagement>
    </ProfileBody>
  </ISO15745Profile>
</ISO15745ProfileContainer>
