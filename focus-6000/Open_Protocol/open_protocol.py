#!/usr/bin/env python
# coding: utf8



class OpCon():
    def __init__(self):
        

        self.MID = {
            'Communication_start': '0001',
            'Communication_start_acknowledge': '0002',
            'Communication_stop': '0003',
            'Command_error': '0004',
            'Command_accepted': '0005',
            'Generic_subscription': '0008',
            'Generic_unsubscribe': '0009',
            'Pset_ID_upload_request': '0010',
            'Pset_ID_upload_reply': '0011',
            'Pset_data_upload_request': '0012',
            'Pset_data_upload_reply': '0013',
            'Pset_selected_subscribe': '0014',
            'Pset_selected': '0015',
            'Pset_selected_acknowledge': '0016',
            'Pset_selected_unsubscribe': '0017',
            'Select_Parameter_set': '0018',
            'Set_Pset_batch_size': '0019',
            'Reset_Pset_batch_counter': '0020',
            'Job_ID_upload_request': '0030',
            'Job_ID_upload_reply': '0031',
            'Tool_data_upload_request': '0040',
            'Tool_data_upload_reply': '0041',
            'Disable_tool': '0042',
            'Enable_tool': '0043',
            'Vehicle_ID_Nr_download_request': '0050',
            'VIN_subscribe': '0051',
            'VIN': '0052',
            'VIN_acknowledge': '0053',
            'VIN_unsubscribe': '0054',
            'Last_tightening_result_data_subscribe': '0060',
            'Last_tightening_result_data': '0061',
            'Last_tightening_result_data_acknowledge': '0062',
            'Last_tightening_result_data_unsubscribe': "0063",
            'Old_tightening_result_upload_request': '0064',
            'Old_tightening_result_upload_reply': '0065',
            'Read_time_upload_request': '0080',
            'Read_time_upload_reply': '0081',
            'Set_time': '0082',
            'Result_traces_curve': '0900',
            'Result_traces_curve_plot_data': '0901',
            'Keep_Alive': '9999',
            'Digital Input Function': '0224',
            'Tightening Program Message download':'2500',
            'Application data message request':'0006',
        }

    def message(self, mid, rev, data, Nof, seqNr, stID="  ", spID="  ", msgPrt=" ", msgNr=" ", NULL="\x00"):
        x = self.MID[mid]
        lng = str(len("0000" + x + rev + Nof + stID + spID + seqNr + msgPrt + msgNr + data))#return the length of the message
        sd = lng.zfill(4) + x + rev + Nof + stID + spID + seqNr + msgPrt + msgNr + data + NULL #the message to be sended to controller 
        return sd.encode()  # convert the message to utf-8,for open protocol the message should be wirtten in utf-8


    def pset_message(self,pset_number,loosening_speed,loosening_torque,loosening_angle,tightening_speed,tightening_torque):
        if pset_number!=1:
          if loosening_speed>=50 and loosening_speed<=750:    
             self.loosening_speed_data='30100201000003020000000'+str(pset_number).zfill(3)+'3001400402101'+'0000'+str(loosening_speed).zfill(4)+'00'
          else:
              print'loosening_speed: the input is outside of the limit'  
          if loosening_torque>=0 and loosening_torque<=300:    
             loosening_torque=format(float(loosening_torque)/100,'.3f')
             self.loosening_torque_data='30100201000003020000000'+str(pset_number).zfill(3)+'3001201290090'+'0000'+'0000'+str(loosening_torque)+'e+3'+'00'
          else:
              print'loosening_torque: the input is outside of the limit'
          if loosening_angle>=0 and loosening_angle<=10800:  
             loosening_angle=format(float(loosening_angle)/1000,'.3f')
             self.loosening_angle_data='30100201000003020000000'+str(pset_number).zfill(3)+'3001301290050'+'0000'+'0000'+str(loosening_angle)+'e+3'+'00'
          else:
              print'loosening_angle: the input is outside of the limit'
          if tightening_speed>=50 and tightening_speed<=100:
             self.tightening_speed_data='30100301000003020000000'+str(pset_number).zfill(3)+'301000010200000013'+'3010100402101'+'0001'+str(tightening_speed).zfill(4)+'00'
          else: 
              print'tightening_speed: the input is outside of the limit'
          if tightening_torque>=62.5 and tightening_torque<=275:     
             tightening_torque=format(float(tightening_torque)/100,'.3f')
             self.tightening_torque_data='30100301000003020000000'+str(pset_number).zfill(3)+'301000010200000013'+'3011201290090'+'0001'+'0000'+str(tightening_torque)+'e+3'+'00'
          else:
              print'tightening_torque: the input is out of the limit'  
          #if tightening_angle>=0 and tightening_angle<=10800:      
          #   tightening_angle=format(float(tightening_angle)/1000,'.3f')
          #   self.tightening_angle_data='30100301000003020000000'+str(pset_number).zfill(3)+'301000010200000012'+'3010901290050'+'0001'+'0000'+str(tightening_angle)+'e+3'+'00'
          #else:
          #    print'tightening_torque: the input is out of the limit')  
        else:
          print'unvalid pset_number'           
    '''
    function to decode the message that sended back by the controller in the process 'Last_tightening_result_data'
    '''
    def decode(self,msg):

        decoded = msg
        mid = decoded[4:8]
        if mid == self.MID['Last_tightening_result_data']:
            '''Decode MID 0061 REV 001'''
            torque_controller_name = "Torque Controller Name: "+decoded[32:57]
            job_id = "Job ID: "+decoded[86:88]
            parameter_set_id = "PSet ID: "+decoded[90:93]
            if decoded[107:108] == "1":
                tightening_status = "Tightening Status: "+"OK"
            elif decoded[107:108] == "0":
                tightening_status = "Tightening Status: "+"NOK"

            if decoded[110:111] == "0":
                torque_status = "Torque Status: LOW"
            elif decoded[110:111] == "1":
                torque_status = "Torque Status: OK"
            elif decoded[110:111] == "2":
                torque_status = "Torque Status: HIGH"

            if decoded[113:114] == "0":
                angle_status = "Angle Status: LOW"
            elif decoded[113:114] == "1":
                angle_status = "Angle Status: OK"
            elif decoded[113:114] == "2":
                angle_status = "Angle Status: HIGH"

            torque = "Torque: "+str(int(decoded[140:146])/100)+" Nm"
            angle = "Angle: "+decoded[169:174]+" deg"
            time_stamp = "Time Stamp: "+decoded[176:195]
            tightening_id = "Tightening ID: "+decoded[221:231]
            data = (torque_controller_name, job_id, parameter_set_id, tightening_status,
                    torque_status, angle_status, torque, angle, time_stamp, tightening_id)
            '''
            0 Torque Controller Name
            1 Job ID
            2 Pset ID
            3 Tightening Status
            4 Torque Status
            5 Angle Status
            6 Torque
            7 Angle
            ....
            '''

        return data
 
