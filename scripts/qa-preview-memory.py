# Run in Rhino 8 with _-RunPythonScript after building/installing Motus.GH.
# Tests native resource ownership without modifying an existing GH document.
import clr
import os
import traceback
import tempfile
import System
import Rhino
from System.Reflection import BindingFlags

LOG = os.path.join(tempfile.gettempdir(), "motus-qa-preview-memory.log")
flags = BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.Public

def field(obj, name):
    t = obj.GetType()
    while t is not None:
        f = t.GetField(name, flags)
        if f is not None:
            return f
        t = t.BaseType
    raise AssertionError("Missing field " + name)

def get(obj, name):
    return field(obj, name).GetValue(obj)

def call(obj, name, *args):
    return obj.GetType().GetMethod(name, flags).Invoke(obj, System.Array[System.Object](args))

def check(condition, message):
    if not condition:
        raise AssertionError(message)

def run():
    Rhino.RhinoApp.RunScript("_Grasshopper", False)
    clr.AddReference("Grasshopper")
    import Grasshopper
    from Grasshopper.Kernel import GH_Document, GH_DocumentContext
    asm = next(a for a in System.AppDomain.CurrentDomain.GetAssemblies() if a.GetName().Name == "Motus.GH")
    def component(name):
        return System.Activator.CreateInstance(asm.GetType("Motus.GH.Components." + name, True))

    doc = GH_Document()
    try:
        # A real solve allocates native geometry. Re-solves must dispose the previous preview,
        # while delete/undo and document close must both release the surviving meshes.
        for name in ["MotusCollisionBoxComponent", "MotusCollisionSphereComponent",
                     "MotusCollisionPlaneComponent", "MotusCollisionSceneComponent",
                     "MotusCollisionBoxesComponent", "MotusUr10eRobotiqComponent"]:
            c = component(name)
            doc.AddObject(c, False)
            if name == "MotusCollisionBoxesComponent":
                from Grasshopper.Kernel.Types import GH_Plane
                c.Params.Input[0].PersistentData.Append(GH_Plane(Rhino.Geometry.Plane.WorldXY))
            if name == "MotusCollisionSceneComponent":
                # The object input is generic; solve the upstream box in this same test document.
                box = component("MotusCollisionBoxComponent")
                doc.AddObject(box, False)
                c.Params.Input[0].AddSource(box.Params.Output[0])
            doc.NewSolution(False)
            meshes = list(get(c, "_previewMeshes"))
            check(len(meshes) > 0, name + " did not create preview meshes")
            if name == "MotusUr10eRobotiqComponent":
                field(c, "_showCollisionPreview").SetValue(c, True)
            else:
                field(c, "_previewKey").SetValue(c, None)
            c.ExpireSolution(False)
            doc.NewSolution(False)
            check(all(m.Disposed for m in meshes), name + " retained replaced native meshes")
            meshes = list(get(c, "_previewMeshes"))
            check(all(not m.Disposed for m in meshes), name + " disposed live output meshes")
            doc.RemoveObject(c, False)
            check(all(m.Disposed for m in meshes), name + " retained removed meshes")
            doc.AddObject(c, False)
            c.ExpireSolution(False)
            doc.NewSolution(False)
            meshes = list(get(c, "_previewMeshes"))
            check(len(meshes) > 0, name + " did not rebuild after undo")
            c.DocumentContextChanged(doc, GH_DocumentContext.Close)
            check(all(m.Disposed for m in meshes), name + " retained meshes on close")
            doc.RemoveObject(c, False)
            print("PASS " + name + " replace/remove/restore/close")

        preview = component("MotusPreviewComponent")
        doc.AddObject(preview, False)
        # Simulate repeated pick/place replans: old local templates, released objects, and
        # working meshes must all be disposed before names/geometry from the next plan arrive.
        for cycle in range(100):
            local = Rhino.Geometry.Mesh()
            released = Rhino.Geometry.Mesh()
            active = Rhino.Geometry.Mesh()
            get(preview, "_attachedLocalCache").Add("brick_" + str(cycle), local)
            key = System.ValueTuple[System.Int32, System.Int32](0, 0)
            get(preview, "_releasedMeshesCache").Add(key, released)
            get(preview, "_activeAttachedMeshes").Add(active)
            get(preview, "_activeAttachedLast").Add(Rhino.Geometry.Transform.Unset)
            get(preview, "_activeAttachedNames").Add("brick_" + str(cycle))
            call(preview, "InvalidateReleasedMeshCache")
            check(local.Disposed and released.Disposed and active.Disposed,
                  "Replan retained attachment meshes")
            check(get(preview, "_attachedLocalCache").Count == 0, "Local templates grew across replans")
        print("PASS 100 attachment cache replacements")
        # Populate ownership slots directly so cleanup is checked even without planning a robot path.
        owned = []
        for slot in ["_currentMeshes", "_startMeshes", "_activeAttachedMeshes"]:
            mesh = Rhino.Geometry.Mesh()
            get(preview, slot).Add(mesh)
            owned.append(mesh)
        for slot in ["_sceneMeshesCache", "_attachedLocalCache"]:
            mesh = Rhino.Geometry.Mesh()
            get(preview, slot).Add("brick", mesh)
            owned.append(mesh)
        # Thousands of custom colours must not create an unbounded material cache.
        from System.Drawing import Color
        for i in range(1024):
            call(preview, "MaterialFor", Color.FromArgb(i % 256, i // 256, 10), System.Single(0.25))
            check(get(preview, "_materialCache").Count <= 128, "Unbounded display materials")
        call(preview, "StartPlayTimer")
        preview.DocumentContextChanged(doc, GH_DocumentContext.Close)
        check(all(m.Disposed for m in owned), "Preview retained meshes on close")
        check(get(preview, "_materialCache").Count == 0, "Preview retained materials on close")
        check(get(preview, "_playTimer") is None, "Preview retained timer on close")
        # Repeated cleanup, then timer recreation, supports removal and undo.
        doc.RemoveObject(preview, False)
        doc.AddObject(preview, False)
        call(preview, "StartPlayTimer")
        check(get(preview, "_playTimer") is not None, "Preview timer did not restart after undo")
        doc.RemoveObject(preview, False)
        print("PASS Preview materials/timer/mesh cleanup")
    finally:
        doc.Dispose()
    print("All preview memory ownership checks passed.")

try:
    # Keep a machine-readable result as well as Rhino command history output.
    run()
    with open(LOG, "w") as output:
        output.write("PASS: all native preview memory ownership checks passed.\n")
except:
    error = traceback.format_exc()
    with open(LOG, "w") as output:
        output.write(error)
    print(error)
    raise
