""" Utilities to export worlds to Ignition Gazebo """

import os
import shutil
import itertools
from shapely.geometry import LineString, Polygon, MultiPolygon
from shapely.ops import split

from .hallway import Hallway
from .room import Room
from ..utils.general import get_data_folder


class WorldGazeboExporter:
    def __init__(self, world):
        self.world = world

        self.data_folder = get_data_folder()
        self.template_folder = os.path.join(self.data_folder, "templates")


    def export(self):
        """ Exports the world to an SDF file to use with Gazebo """
        world_name = self.world.name
        if world_name is None:
            world_name = "gen_world"

        # Set up template text
        world_text = self.read_template_file("world_template.sdf")
        model_template_text = self.read_template_file("model_template.sdf")
        model_config_template_text = self.read_template_file("model_template.config")
        link_template_text = self.read_template_file("link_template_polyline.sdf")

        # Define output folder
        out_folder = os.path.join(self.data_folder, "worlds", world_name)


        model_include_text = ""

        # Convert all room / hallway polygons
        walls_name = "walls"
        full_links_text = ""
        for obj in itertools.chain(self.world.rooms, self.world.hallways):
            full_links_text += self.create_sdf_link_text(
                link_template_text, obj)
        
        # Now replace the main model SDF and tack on the links text
        walls_text = model_template_text
        walls_text = walls_text.replace("$NAME", walls_name)
        walls_text = walls_text.replace("$POSE", "0 0 0 0 0 0")
        walls_text = walls_text.replace("$STATIC", "1")
        walls_text = walls_text.replace("$LINKS", full_links_text)
        walls_folder = os.path.join(out_folder, "walls")
        if os.path.isdir(out_folder):
            shutil.rmtree(out_folder)
        os.makedirs(walls_folder)
        with open(os.path.join(out_folder, walls_name, "model.sdf"), "w") as f:
            f.write(walls_text)

        # Now include
        model_include_text += " "*4 + "<include>\n"
        model_include_text += " "*8 + f"<uri>model://{walls_name}</uri>\n"
        model_include_text += " "*4 + "</include>\n"

        config_text = model_config_template_text.replace("$NAME", world_name)
        with open(os.path.join(out_folder, walls_name, "model.config"), "w") as f:
            f.write(config_text)

        # Export locations and objects
        for entity in itertools.chain(self.world.locations, self.world.objects):
            # Add the category if not existing
            # If the object does not have meshes, create a new model and extrude a polyline.
            # If it does have meshes, the model can be included as is.
            if entity.metadata["footprint"]["type"] != "mesh":
                folder = os.path.join(out_folder, entity.category)
                if not os.path.isdir(folder):
                    os.makedirs(folder)

                    loc_model_text = model_template_text
                    loc_model_text = loc_model_text.replace("$NAME", entity.category)
                    loc_model_text = loc_model_text.replace("$POSE", "0 0 0 0 0 0")
                    loc_link_text = self.create_sdf_link_text(link_template_text, entity)
                    loc_model_text = loc_model_text.replace("$LINKS", loc_link_text)
                    loc_model_text = loc_model_text.replace("$STATIC", "0")

                    with open(os.path.join(out_folder, entity.category, "model.sdf"), "w") as f:
                        f.write(loc_model_text)

                    config_text = model_config_template_text.replace("$NAME", entity.category)
                    with open(os.path.join(out_folder, entity.category, "model.config"), "w") as f:
                        f.write(config_text)

            if entity.metadata["footprint"]["type"] == "mesh":
                model_path = entity.metadata["footprint"]["model_path"]
                model_name = model_path.split(os.sep)[-1]
            else:
                model_name = entity.category

            # Now add the instance
            model_include_text += " "*4 + "<include>\n"
            model_include_text += " "*8 + f"<uri>model://{model_name}</uri>\n"
            model_include_text += " "*8 + f"<name>{entity.name}</name>\n"
            pose_str = f"{entity.pose.x} {entity.pose.y} {entity.pose.z} 0 0 {entity.pose.yaw}"
            model_include_text += " "*8 + f"<pose>{pose_str}</pose>\n"
            model_include_text += " "*4 + "</include>\n"


        # Wrap up
        world_text = world_text.replace("$MODEL_INCLUDES", model_include_text)
        world_file_name = os.path.join(out_folder, f"{world_name}.sdf")
        with open(world_file_name, "w") as f:
            f.write(world_text)
        print(f"\nWorld file saved to {world_file_name}\n")
        print(f"Ensure to update your Gazebo model path:")
        print(f"    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:{out_folder}\n")
        print(f"To start the world, enter")
        print(f"    gazebo {world_file_name}\n")
        return      


    def create_sdf_link_text(self, template_text, entity):
        """ Creates SDF link text from a world entity """

        # Check the height
        if isinstance(entity, Room) or isinstance(entity, Hallway):
            height = self.world.wall_height
            poly = entity.viz_polygon
        else:
            height = entity.height
            poly = entity.get_raw_polygon()

        # Convert everything to a MultiPolygon for consistency
        if isinstance(poly, Polygon):
            polys = MultiPolygon([poly])
        else:
            polys = poly
        polys = [g for g in polys.geoms]

        # If the polygon is a closed ring (i.e. has interiors), split it into
        # two parts along the bounds diagonal to work in Gazebo
        # NOTE: This was done since using full closed polygons causes errors in Gazebo
        for poly in polys:
            if len(poly.interiors) > 0:
                xmin, ymin, xmax, ymax = poly.bounds
                div_line = LineString([(xmin-1,ymin-1), (xmax+1,ymax+1)])
                new_polys = split(poly, div_line)
                polys.remove(poly)
                polys.extend([p for p in new_polys])

        # Now create the SDF text for each polygon in the list
        full_text = ""
        color_str = " ".join([str(c) for c in entity.viz_color])
        for i, poly in enumerate(polys):
            link_text = template_text
            link_name = entity.name
            if i > 0:
                link_name += f"_{i}"
            link_text = link_text.replace("$LINK_NAME", link_name)
            link_text = link_text.replace("$HEIGHT", f"{height:.3}")
            
            # Write all the polygon coordinates to SDF
            wall_points = ""
            for p in poly.exterior.coords:
                wall_points += " "*12 + f"<point>{p[0]:.3} {p[1]:.3}</point>\n"
            link_text = link_text.replace("$POINTS", wall_points)
            
            # Set the wall color
            link_text = link_text.replace("$COLOR", color_str)

            # Add the individual link's text to the full text
            full_text += link_text
        
        return full_text


    def read_template_file(self, filename):
        """ Utility to read a file from the template folder """
        fullfile = os.path.join(self.template_folder, filename)
        with open(fullfile, "r") as f:
            return f.read()  
