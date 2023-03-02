
import numpy as np
import rospy
from os.path import dirname, realpath
from std_msgs.msg import String, Bool
from armor_msgs.msg import * 
from armor_msgs.srv import * 
from erl2.srv import Consistent
from erl2.msg import ErlOracle
from erl2.srv import Oracle, Consistent

people = ["missScarlett", "colonelMustard", "mrsWhite", "mrGreen", "mrsPeacock", "profPlum"]
weapons = ["candlestick", "dagger", "leadPipe", "revolver", "rope", "spanner"]
places = ["conservatory", "lounge", "kitchen", "library", "hall", "study", "bathroom", "diningRoom", "billiardRoom"]

armor_service = None
hint_pub = None
oracle_client = rospy.ServiceProxy('/oracle_solution', Oracle)
cIDs = []   # store consistent IDs

ID0 = []
ID1 = []
ID2 = []
ID3 = []
ID4 = []
ID5 = []

class Armor_communication():
    """Armor communication class
 
    the class defines all the functions that allow the communcation 
    with the Armor server
    """
    def __init__(self):
        """
        \brief Initilize the class by declaring the client to "armor_interface_srv" service
        """
        super(Armor_communication, self).__init__()
        
        
        print('armor server ready')
        self.armor_service = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
        print('waiting for armor server')
        rospy.wait_for_service('armor_interface_srv')


    def load_file(self):
        """
        \brief loads the owl file
        """
        print('loading owl file...')
        try:
            path = dirname(realpath(__file__))
            req=ArmorDirectiveReq()
            req.client_name= 'cluedo'
            req.reference_name= 'cluedontology'
            req.command= 'LOAD'
            req.primary_command_spec= 'FILE'
            req.secondary_command_spec= ''
            req.args= [path + '/../cluedo_ontology.owl', 'http://www.emarolab.it/cluedo-ontology', 'true', 'PELLET', 'true']
            msg = self.armor_service(req)
            print(msg)
        except rospy.ServiceException as e:
            print(e)
            
    def log_to_file(self):
        """
        \brief logs to a file in the same folder
        """
        print("logging init...")
        try:
            req=ArmorDirectiveReq()
            req.client_name= 'cluedo'
            req.reference_name= 'cluedontology'
            req.command= 'LOG'
            req.primary_command_spec= 'FILE'
            req.secondary_command_spec= 'ON'
            req.args= ['/root/ros_ws/src/erl2/armor_log.txt']
            msg = self.armor_service(req)
            print(msg.armor_response.success)
        except rospy.ServiceException as e:
            print(e)
            
    def load_hints(self):
        """
        \brief loads provided hints on the ontology
        """
        global people, weapons, places
        print("loading hints...")
        try:
            req = ArmorDirectiveReq()
            req.client_name= 'cluedo'
            req.reference_name= 'cluedontology'
            req.command = 'ADD'
            req.primary_command_spec = 'IND'
            req.secondary_command_spec = 'CLASS'
            
            # Upload people list
            for person in people:
                req.args = [person, 'PERSON']
                msg = self.armor_service(req)
                res = msg.armor_response
                print(res)
            # Upload weapons list
            for weapon in weapons:
                req.args = [weapon, 'WEAPON']
                msg = self.armor_service(req)
                res = msg.armor_response
                print(res)
            # Upload places list
            for place in places:
                req.args = [place, 'PLACE']
                msg = self.armor_service(req)
                res = msg.armor_response
                print(res)
        except rospy.ServiceException as e:
            print(e)
        
        print("done")
        
            
    
    
    def instances_disjoint(self):
        """
        \brief Disjoint request for the classes in the ontology
        """
        print("disjoit instances...")
        try:
            req = ArmorDirectiveReq()
            req.client_name= 'cluedo'
            req.reference_name= 'cluedontology'
            req.command = 'DISJOINT'
            req.primary_command_spec = 'IND'
            req.secondary_command_spec = 'CLASS'
            req.args = ['PERSON']
            msg = self.armor_service(req)
            res = msg.armor_response
            req.args = ['WEAPON']
            msg = self.armor_service(req)
            res = msg.armor_response
            req.args = ['PLACE']
            msg = self.armor_service(req)
            res = msg.armor_response
        except rospy.ServiceException as e:
            print(e)
        print("done")

    def reason(self):
        """
        \brief Makes the armor system reason
        """
        print("reasoning...")
        try:
            req=ArmorDirectiveReq()
            req.client_name= 'cluedo'
            req.reference_name= 'cluedontology'
            req.command= 'REASON'
            req.primary_command_spec= ''
            req.secondary_command_spec= ''
            req.args= []
            msg = self.armor_service(req)
        except rospy.ServiceException as e:
            print(e)
        print("done")
            
    def retrieve_class(self,cls):
        """
        \brief Obtain the list of all the places inside the system
        @param cls Class to be retrived
        @return The array requested
        """
        try:
            req=ArmorDirectiveReq()
            req.client_name= 'cluedo'
            req.reference_name= 'cluedontology'
            req.command= 'QUERY'
            req.primary_command_spec= 'IND'
            req.secondary_command_spec= 'CLASS'
            req.args= [cls]
            msg = self.armor_service(req)
            #res = msg.armor_response
            #return res
            queries=msg.armor_response.queried_objects
            cont=0
            A=[0]*len(queries)
            for query in queries:
                results=query[40:]
                results=results[:len(results)-1]
                #print(results)
                A[cont]=results
                cont=cont+1
            return A
        except rospy.ServiceException as e:
            print(e)
            
        
        
    def make_hypothesis(self,hyp,hypID):
        """
        \brief Create an hypothesis in the system
        @param hyp the hypothesis that will be inserted in the system, which is an array
            of various lenght
        @param hypID ID of the hypothesis to be created
        """
        c = 0
        # counts the hints in the hypothesis list
        global people, weapons, places
        for i in range(len(hyp)):
            if hyp[i]:
                c = c+1
        
        for i in range(c):
            try:
                req=ArmorDirectiveReq()
                req.client_name= 'cluedo'
                req.reference_name= 'cluedontology'
                req.command= 'ADD'
                req.primary_command_spec= 'OBJECTPROP'
                req.secondary_command_spec= 'IND'  
                if hyp[i] in people:
                    req.args= ['who', hypID, hyp[i]]
                elif hyp[i] in weapons:
                    req.args= ['what', hypID, hyp[i]]
                elif hyp[i] in places:
                    req.args= ['where', hypID, hyp[i]]
                self.armor_service(req)
                
            except rospy.ServiceException as e:
                print(e)
    
            try:
                req=ArmorDirectiveReq()
                req.client_name= 'cluedo'
                req.reference_name= 'cluedontology'
                req.command= 'ADD'
                req.primary_command_spec= 'IND'
                req.secondary_command_spec= 'CLASS'
                if hyp[i] in people:
                    req.args= [hyp[i],'PERSON']
                elif hyp[i] in weapons:
                    req.args= [hyp[i],'WEAPON']
                elif hyp[i] in places:
                    req.args= [hyp[i],'PLACE']
                self.armor_service(req)
                

            except rospy.ServiceException as e:
                print(e)
        print('Hypothesis loaded')
                

armor = Armor_communication()

def hintCallback(hint):
    global ID0,ID1,ID2,ID3,ID4,ID5, hint_pub
    IDs = [ID0,ID1,ID2,ID3,ID4,ID5]
    print('message received')
    
    # check if ID is correct: not empty and not -1
    if str(hint.ID)=="" or hint.ID == -1:
        print('id wrong')
        hint_pub.publish(False)
        return 
	    
	# check if the key is correct
    if  hint.key not in ["who","what","where"]:
        print('key wrong')
        hint_pub.publish(False)
        return 
	    
	# check if the value is correct: not empty, 0, -1 
    if hint.key == "" or hint.key == -1:
        print('value wrong')
        hint_pub.publish(False)
        return 
	    
    # check if the received hint is new
    if hint.value in IDs[hint.ID]:
        print("I already got this hint")
        hint_pub.publish(False)
    else:
        # store hint in ID arrays
        IDs[hint.ID].append(hint.value)
        print("ID0 ",ID0)
        print("ID1 ",ID1)
        print("ID2 ",ID2)
        print("ID3 ",ID3)
        print("ID4 ",ID4)
        print("ID5 ",ID5)    
        hint_pub.publish(True)
    return

def check_consistency():
    global ID0, ID1, ID2, ID3, ID4, ID5,cIDs
    IDs = [ID0,ID1,ID2,ID3,ID4,ID5]
    
    consistent = False
    print('Are any ID source consistent?')
    print("ID0 ",ID0)
    print("ID1 ",ID1)
    print("ID2 ",ID2)
    print("ID3 ",ID3)
    print("ID4 ",ID4)
    print("ID5 ",ID5)    
    
    cIDs = []   # store consistent IDs
    for i in range(0,5):
        # Check which ID has 3 or more hints
        if len(IDs[i]) >= 3:
            print("id ",i," has ",len(IDs[i])," hints!")
            hyp_list = IDs[i]
            hypothesis_code = str(i)
            # upload hypothesis on ARMOR
            armor.make_hypothesis(hyp_list, hypothesis_code)
            armor.reason()
            # Check consistency
            compl = armor.retrieve_class('COMPLETED')
            incons = armor.retrieve_class('INCONSISTENT') 
            
            rospy.sleep(1)
            if hypothesis_code in compl and hypothesis_code not in incons: 
                print('The hypothesis is consistent')
                consistent = True
                cIDs.append(i)
            else:
                print('The hypothesis is inconsistent')
                
    if consistent:
        return True
    else:
        return False

## id_to_natural_language
#  Given the ID it returns the hints in type string
#  in natural language
def id_to_natural_language(id):
    global ID0, ID1, ID2, ID3, ID4, ID5
    IDs = [ID0,ID1,ID2,ID3,ID4,ID5]
    per = ""
    wea = ""
    pla = ""
    for i in range(0,len(IDs[id])):
        if IDs[id][i] in people:
            per = IDs[id][i]
        elif IDs[id][i] in weapons:
            wea = IDs[id][i]
        elif IDs[id][i] in places:
            pla = IDs[id][i]
            
    msg = per+" with the "+wea+" in the "+pla
    return msg

## Check_consistency
#
#   Callback for the service server /ask_solution.
#   Gets the IDs relative to consistent hypothesis and calls for the /oracle_solution 
#   server to compare them with the winning ID. If one of the ID is the same as the 
#   winning ID it returns a "true" response
def check_result():
    global oracle_client, cIDs
    
    # call /oracle_solution server
    res = oracle_client()
    print(len(cIDs))
    for i in range(0,len(cIDs)):
        # Print query in natural language
        id = cIDs[i]
        msg = id_to_natural_language(id)
        print("Was it ",msg,"?")
        
        if id == res.ID:
            print("I have the correct solution!")
            print("ID: ",id)
            # print solution in natural language
            msg = id_to_natural_language(id)
            print("It was ",msg,"!")
            return True
        else:
            print("This hypothesis is wrong!")
    
    # if it is not returned yet all hyp are wrong
    return False


def main():
    global  armor_service, pub, hint_pub, ID0,ID1,ID2,ID3,ID4,ID5
    rospy.init_node('hint')
    people_list = []
    weapons_list = []
    places_list = []

     # Init OWL file from Armor server
    
    armor.__init__()
    #armor.log_to_file()
    armor.load_file()
    #armor.load_hints()
    armor.instances_disjoint()
    armor.reason()
   
    

    
    ID1 = []
    ID0 = ["missScarlett","library","dagger"]
    ID2 = ["missScarlett"]
    ID3 = ["colonelMustard","mrsWhite","library"]
    ID4 = ["conservatory","dagger", "leadPipe", "revolver"]
    ID5 = []

    print("CONSISTENCY")
    check_consistency()
    print("RESULT")
    check_result()
    
  
    print('Initialization completed')
    
    rospy.spin() 

if __name__ == '__main__':
  main()        
