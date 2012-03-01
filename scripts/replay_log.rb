require 'orocos'
require 'vizkit'
include Orocos
Orocos.initialize

if ARGV[0] == nil
	puts "Please add a path to a logfile, containing sonarscans, as argument!"
	exit
end

view3d = Vizkit.default_loader.create_widget('vizkit::QVizkitMainWindow')
view3d.show
sonarbeamviz = view3d.createPlugin("sonarbeam","SonarBeamVisualization")

log = Orocos::Log::Replay.open(ARGV)

log.sonar.BaseScan  do |sample|
    sonarbeamviz.updateSonarBeam(sample)
    sample
end

#log.orientation_estimator.orientation_samples  do |sample|
#    sonarbeamviz.updateBodyState(sample)
#    sample
#end 

Vizkit.control log
Vizkit.exec 
