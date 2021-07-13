import urllib2
import osmapi
def ParseOsmFile(outputFile='map.osm', inputOsmFile=''):
    '''downloads the data file for the specified bounding box
       stores the file as outputFile, if inputOsmFile is not specified
       and also converts the data in the form of a dictionary'''
    if not inputOsmFile:
        return None
    else:
        dataDict = {}
        print(inputOsmFile)
        osmRead = open(inputOsmFile, 'r')
        myapi = osmapi.OsmApi()
        dataDict = myapi.ParseOsm(osmRead.read())
        osmRead.close()
        print(dataDict)
        return dataDict
#a=ParseOsmFile( outputFile='map.osm', inputOsmFile='berlin_osm.osm')

