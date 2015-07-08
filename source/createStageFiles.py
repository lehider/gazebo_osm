import os

class StageWorld:

	def __init__(self, mapSize, imageSize, startCoordinates):
		self.xSize = mapSize[0]
		self.ySize = mapSize[1]
		self.zSize = 4 # high enough so laser detects walls

		self.imgWidth = imageSize[0]
		self.imgHeight = imageSize[1]

		self.startLat = startCoordinates[0]
		self.startLon = startCoordinates[1]

	def createStageSetup(self, outputName):
		#outputName = "poplar_ave"
		tempOut = "../worlds/stage_ros/" + outputName + "/"
		self.createStageWorldFile(outputName, tempOut)
		self.createStageYamlFile(outputName,tempOut)


	def createStageWorldFile(self, outputName, outputDir):
		if not os.path.isdir(outputDir):
			os.mkdir(outputDir)

		# grab map.world template
		tempWorld = open("templates/map.world-template", "r")
		worldTemplate = tempWorld.read()

		fo = open(outputDir + outputName + ".world", "w") # open file for writing, create if not open
		fo.write(worldTemplate)
		fo.write("\nsize [ " + str(self.xSize) + " " + str(self.ySize) + " " + str(self.zSize) + " ]    #Map Size\n\n")

		fo.write("map\n(\n")
		fo.write("  gui_grid 0\n")
		fo.write("  bitmap " + outputName + ".png\n")
		fo.write("  size [ " + str(self.imgWidth) + " " + str(self.imgHeight) + " 0.1 ]\n")
		fo.write("  name \"" + outputName + "\"\n")
		fo.write("  obstacle_return 0\n")
		fo.write("  laser_return 0\n)\n\n")

		fo.write("car\n(\n")
		fo.write("  name \"marvin\"\n")
		fo.write("  color \"purple\"\n")
		fo.write("  pose [ " + str(self.startLat) + " " + str(self.startLon) + " " + " 0 180.0]\n")
		fo.write("  localization_origin [0.0 0.0 0.0 0.0 0.0 0.0]\n")
		fo.write("  localization \"gps\" \n")
		fo.write("  sick_laser( samples 181 origin [3.178 0.0 -0.94 0.0] )\n")
		fo.write("  laser_return 1          # visible to lasers\n)\n\n")
		fo.close()

	def createStageYamlFile(self, outputName, outputDir):
		#create stage world file should have already created the dir if necessary
		print "YAML..."

if __name__ == "__main__":

	world = StageWorld([443, 700], [1300, 4800], [-45.12, 14.4334])

	world.createStageSetup("street1")