### Service Access Points (SAP) & Primitives

***Reference from IEEE 1609.3, 2016***

| SAP  | Primitive                              | Specified in | Support | Remarks    |
| ---- | -------------------------------------- | ------------ | ------- | ---------- |
| WSM  | WSM-WaveShortMessage.request           | IEEE 1609.3  |         |            |
|      | WSM-WaveShortMessage.confirm           |              |         |            |
|      | WSM-WaveShortMessage.indication        |              |         |            |
| WME  | WME-ProviderService.request            |              |         |            |
|      | WME-ProviderService.confirm            |              |         |            |
|      | WME-UserService.request                |              |         |            |
|      | WME-UserService.confirm                |              |         |            |
|      | WME-ChannelService.request             |              |         |            |
|      | WME-ChannelService.confirm             |              |         |            |
|      | WME-TimingAdvertisementService.request |              |         |            |
|      | WME-TimingAdvertisementService.confirm |              |         |            |
|      | WME-Notification.indication            |              |         |            |
|      | WME-Get.request                        |              |         |            |
|      | WME-Get.confirm                        |              |         |            |
|      | WME-Set.request                        |              |         |            |
|      | WME-Set.confirm                        |              |         |            |
|      | WME-AddressChange.request              |              |         |            |
|      | WME-AddressChange.confirm              |              |         |            |
| LSAP | DL-UNITDATA.request                    | IEEE 802.2   |         | IPv6       |
|      | DL-UNITDATA.indication                 |              |         |            |
|      | DL-UNITDATAX.request                   | IEEE 1609.3  |         | WSMP       |
|      | DL-UNITDATAX.indication                |              | O       |            |
| MLME | MLMEX-DELETETXPROFILE                  | IEEE 1609.4  |         |            |
|      | MLMEX-REGISTERTXPROFILE                |              |         |            |
|      | MLMEX-TA                               |              |         |            |
|      | MLMEX-TAEND                            |              |         |            |
|      | MLMEX-CHSTART                          |              |         |            |
|      | MLMEX-CHEND                            |              |         |            |
|      | MLME-GET                               | IEEE 802.11  |         |            |
|      | MLME-SET                               |              |         |            |
| MAC  | MA-UNITDATA                            |              |         |            |
|      | MA-UNITDATAX                           | IEEE 1609.4  |         |            |
| Sec  | Sec-SignedData                         | IEEE 1609.2  |         |            |
|      | Sec-SignedDataVerification             |              |         |            |
|      | Sec-SecureDataPreprocessing            |              |         |            |

