'''Description: '''


def refineTargets(target_arr, f):
    refined_targets = []
    for set_of_targets in target_arr:
        if len(set_of_targets) > 4:
            refined_targets.append(set_of_targets)
    for set_of_targets in refined_targets:
        lat_sum = 0
        lon_sum = 0
        count_type_target={'frowny':0, 'smiley':0, 'tarp':0}

        for target in set_of_targets:
            lat_sum += target['lat']
            lon_sum += target['lon']
            count_type_target[target['type']] += 1

        avg_lat = lat_sum/len(set_of_targets)
        avg_lon = lon_sum/len(set_of_targets)


        f.write(max(count_type_target, key=count_type_target.get)+'  lat='+str(avg_lat)+' , lon='+str(avg_lon)+'\n')