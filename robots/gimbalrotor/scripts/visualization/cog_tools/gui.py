"""Qt controls; all session mutations and widget updates happen on the Qt thread."""
import copy
import json
import math
from pathlib import Path
import time
import numpy as np
import yaml
from python_qt_binding import QtCore, QtWidgets
from .math3d import transform, vector, import_properties
from .reference import Registry, placement
from .model import load_model


def rpy_of(t):
    r = t[:3, :3]
    p = math.asin(np.clip(-r[2,0], -1, 1))
    if abs(math.cos(p)) > 1e-8:
        return [math.atan2(r[2,1], r[2,2]), p, math.atan2(r[1,0], r[0,0])]
    return [0, p, math.atan2(-r[0,1], r[1,1])]


def text_vector(value):
    return ' '.join('%.12g' % v for v in value)


def read_vector(edit):
    return vector(edit.text().replace(',', ' ').split())


class Window(QtWidgets.QMainWindow):
    def __init__(self, runtime):
        super().__init__()
        self.rt, self.s, self.refreshing = runtime, runtime.session, False
        self.setWindowTitle('Beetle module / frame / CoG inspector — read-only robot observer')
        self.resize(1150, 800)
        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(central)
        self.setCentralWidget(central)
        model_row = QtWidgets.QHBoxLayout()
        self.preset = QtWidgets.QComboBox()
        self.preset.addItems(list(runtime.presets['presets']))
        self.preset.setCurrentText(runtime.preset_name)
        self.source = QtWidgets.QLineEdit(runtime.source)
        self.source.setToolTip('param:/gimbalrotor/robot_description, absolute expanded URDF, or top-level .xacro')
        model_row.addWidget(self.preset)
        model_row.addWidget(self.source, 1)
        self.button(model_row, 'Browse model', self.browse_model)
        self.button(model_row, 'Load / reload model', self.reload_model)
        layout.addLayout(model_row)
        pose_row = QtWidgets.QHBoxLayout()
        self.pose_mode = QtWidgets.QComboBox()
        self.pose_mode.addItems(['preview', 'live'])
        self.pose_mode.setCurrentText(self.s.pose_source)
        self.frame = QtWidgets.QLineEdit('gimbalrotor/base_link' if self.s.pose_source == 'preview' else self.s.fixed_frame)
        self.prefix = QtWidgets.QLineEdit(self.s.tf_prefix)
        self.frame.setMaximumWidth(230)
        self.prefix.setMaximumWidth(150)
        pose_row.addWidget(QtWidgets.QLabel('Pose source'))
        pose_row.addWidget(self.pose_mode)
        pose_row.addWidget(QtWidgets.QLabel('Live reference frame'))
        pose_row.addWidget(self.frame)
        pose_row.addWidget(QtWidgets.QLabel('TF prefix'))
        pose_row.addWidget(self.prefix)
        self.button(pose_row, 'Apply pose source', self.set_mode)
        self.button(pose_row, 'Copy live to preview', self.copy_live)
        layout.addLayout(pose_row)
        self.banner = QtWidgets.QLabel('')
        self.banner.setWordWrap(True)
        layout.addWidget(self.banner)
        splitter = QtWidgets.QSplitter()
        layout.addWidget(splitter, 1)
        left = QtWidgets.QWidget()
        ll = QtWidgets.QVBoxLayout(left)
        self.search = QtWidgets.QLineEdit()
        self.search.setPlaceholderText('Search links')
        self.search.textChanged.connect(self.filter_tree)
        ll.addWidget(self.search)
        self.tree = QtWidgets.QTreeWidget()
        self.tree.setHeaderLabels(['Link / component', 'Geometry', 'Mass'])
        self.tree.setIndentation(12)
        self.tree.header().setStretchLastSection(False)
        self.tree.header().setSectionResizeMode(0,QtWidgets.QHeaderView.Stretch)
        self.tree.header().setSectionResizeMode(1,QtWidgets.QHeaderView.ResizeToContents)
        self.tree.header().setSectionResizeMode(2,QtWidgets.QHeaderView.ResizeToContents)
        self.tree.setMinimumWidth(360)
        self.tree.itemSelectionChanged.connect(self.select_link)
        self.tree.itemChanged.connect(self.membership_changed)
        ll.addWidget(self.tree)
        for names in [('Start assembly from selection', self.start_assembly), ('Isolate geometry', self.isolate),
                      ('Show all geometry', self.show_all), ('All declared URDF masses (includes helpers)', self.all_masses),
                      ('Add parent', lambda: self.add_adjacent('parent')), ('Add child', lambda: self.add_adjacent('child')),
                      ('Add connected module', lambda: self.add_adjacent('both')), ('Add chain to…', self.add_chain)]:
            self.button(ll, *names)
        self.chain_masses = QtWidgets.QCheckBox('Include connector masses when adding a chain')
        self.chain_masses.setChecked(True)
        ll.addWidget(self.chain_masses)
        self.group_choice = QtWidgets.QComboBox()
        ll.addWidget(self.group_choice)
        self.button(ll, 'Add preset group masses + geometry', self.add_group)
        self.action_status = QtWidgets.QLabel('Geometry visibility and mass membership are independent.')
        self.action_status.setWordWrap(True)
        ll.addWidget(self.action_status)
        splitter.addWidget(left)
        self.tabs = QtWidgets.QTabWidget()
        splitter.addWidget(self.tabs)
        splitter.setStretchFactor(1, 3)
        audit = QtWidgets.QWidget()
        audit_layout = QtWidgets.QVBoxLayout(audit)
        self.audit_table = QtWidgets.QTableWidget(0, 9)
        self.audit_table.setHorizontalHeaderLabels(['Link', 'Variant', 'Source', 'Mass kg', 'Local CoG m', 'Fixed CoG m', 'Δ mm', 'Calibration', 'Diagnostics'])
        self.audit_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        for column,width in enumerate([185,80,90,85,220,220,70,250,300]):
            self.audit_table.setColumnWidth(column,width)
        audit_layout.addWidget(self.audit_table, 1)
        self.numeric = QtWidgets.QPlainTextEdit()
        self.numeric.setReadOnly(True)
        audit_layout.addWidget(self.numeric, 1)
        self.tabs.addTab(audit, 'Mass / frames / audit')
        self.joint_panel = QtWidgets.QWidget()
        self.joint_layout = QtWidgets.QFormLayout(self.joint_panel)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(self.joint_panel)
        self.tabs.addTab(scroll, 'Joint positions')
        self.make_edit_panel()
        self.make_reference_panel()
        self.make_display_panel()
        export_row = QtWidgets.QHBoxLayout()
        self.button(export_row, 'Save session + reference + reports + origin suggestions', self.export)
        self.button(export_row, 'Load saved session', self.restore)
        self.button(export_row, 'Import reference YAML', self.import_reference)
        layout.addLayout(export_row)
        self.rebuild()
        self.last_audit = 0
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(round(1000/runtime.rate))

    def safe(self, callback):
        def invoke(*args):
            try:
                callback()
            except Exception as exc:
                self.s.last_error = str(exc)
                QtWidgets.QMessageBox.warning(self, 'Action unavailable', str(exc))
        return invoke

    def button(self, layout, label, callback):
        b = QtWidgets.QPushButton(label)
        b.clicked.connect(self.safe(callback))
        layout.addWidget(b)
        return b

    def line(self, form, label, value='0 0 0'):
        edit = QtWidgets.QLineEdit(value)
        form.addRow(label, edit)
        return edit

    def scroll_tab(self, panel, title):
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(panel)
        self.tabs.addTab(scroll,title)

    def make_edit_panel(self):
        self.edit_panel = QtWidgets.QWidget()
        f = QtWidgets.QFormLayout(self.edit_panel)
        f.addRow(QtWidgets.QLabel('Preview overrides are absolute link/joint-local transforms. RPY in radians.'))
        self.visual_index = QtWidgets.QComboBox()
        self.visual_index.currentIndexChanged.connect(lambda _: self.load_visual_fields())
        f.addRow('Selected visual', self.visual_index)
        self.visual_xyz = self.line(f, 'Visual XYZ [m]')
        self.visual_rpy = self.line(f, 'Visual RPY [rad]')
        buttons = QtWidgets.QHBoxLayout()
        self.button(buttons, 'Apply visual-only override', self.apply_visual)
        self.button(buttons, 'Reset this visual', self.reset_visual)
        f.addRow(buttons)
        self.origin_joint = QtWidgets.QComboBox()
        self.origin_joint.currentTextChanged.connect(lambda _: self.load_origin_fields())
        f.addRow('Joint origin to override', self.origin_joint)
        self.origin_xyz = self.line(f, 'Joint origin XYZ [m]')
        self.origin_rpy = self.line(f, 'Joint origin RPY [rad]')
        buttons = QtWidgets.QHBoxLayout()
        self.button(buttons, 'Apply joint-origin override', self.apply_origin)
        self.button(buttons, 'Reset this origin', self.reset_origin)
        self.button(buttons, 'Reset all preview overrides', self.reset_overrides)
        f.addRow(buttons)
        self.scroll_tab(self.edit_panel, 'Preview origins')

    def make_reference_panel(self):
        panel = QtWidgets.QWidget()
        f = QtWidgets.QFormLayout(panel)
        warning = QtWidgets.QLabel('Import original CAD coordinates explicitly. Source, frame relationship, and placement have separate trust states. Applying creates an explicit viewer record revision; model reload never reseeds it.')
        warning.setWordWrap(True)
        f.addRow(warning)
        self.ref_mass = self.line(f, 'Original mass', '')
        self.mass_unit = QtWidgets.QComboBox()
        self.mass_unit.addItems(['kg','g'])
        f.addRow('Mass units', self.mass_unit)
        self.ref_cog = self.line(f, 'Original CAD CoG XYZ', '')
        self.length_unit = QtWidgets.QComboBox()
        self.length_unit.addItems(['m','mm'])
        f.addRow('CoG units', self.length_unit)
        self.ref_ma_known = QtWidgets.QCheckBox('CAD → mesh relationship available (otherwise no candidate/ghost)')
        f.addRow(self.ref_ma_known)
        self.ref_ma_xyz = self.line(f, 'T_M_A CAD → normalized mesh XYZ [m]')
        self.ref_ma_rpy = self.line(f, 'T_M_A RPY [rad]')
        self.ref_ma_known.toggled.connect(self.ref_ma_xyz.setEnabled)
        self.ref_ma_known.toggled.connect(self.ref_ma_rpy.setEnabled)
        self.ref_la_xyz = self.line(f, 'Saved T_L_A CAD → link XYZ [m]')
        self.ref_la_rpy = self.line(f, 'Saved T_L_A RPY [rad]')
        self.ref_note = self.line(f, 'CAD assembly / revision / provenance', '')
        self.ref_statuses = []
        for label in ['Mass source trust', 'CAD/mesh frame trust', 'Placement trust']:
            c = QtWidgets.QComboBox()
            c.addItems(['assumed', 'verified'])
            f.addRow(label, c)
            self.ref_statuses.append(c)
        self.ref_complete = QtWidgets.QCheckBox('Selected visual is the entire rigid mass record')
        self.ref_follow = QtWidgets.QCheckBox('Allow mesh-follow candidate (assumption)')
        f.addRow(self.ref_complete)
        f.addRow(self.ref_follow)
        self.threshold = QtWidgets.QDoubleSpinBox()
        self.threshold.setDecimals(3)
        self.threshold.setRange(0,10000)
        self.threshold.setValue(self.s.threshold_m*1000)
        self.threshold.valueChanged.connect(lambda value: setattr(self.s,'threshold_m',value/1000))
        f.addRow('Discrepancy threshold [mm]',self.threshold)
        self.button(f, 'Apply original values + frame calibration to session', self.apply_reference)
        self.button(f, 'Accept current mesh placement as saved reference (assumed)', self.accept_placement)
        self.button(f, 'Reset references to original imported baseline', self.reset_references)
        self.button(f, 'Save viewer reference YAML', self.save_reference)
        self.scroll_tab(panel, 'CAD import / calibration')

    def make_display_panel(self):
        panel = QtWidgets.QWidget()
        f = QtWidgets.QFormLayout(panel)
        self.overlay_widgets, self.display_widgets = {}, {}
        for key, enabled in self.s.overlays.items():
            c = QtWidgets.QCheckBox(key)
            c.setChecked(enabled)
            c.toggled.connect(lambda value, k=key: self.s.overlays.__setitem__(k, value))
            self.overlay_widgets[key] = c
            f.addRow(c)
        for key, label, maximum in [('explosion','Display-only X separation [m]', 2), ('alpha','Mesh opacity', 1), ('axis_length','Axis length [m]', .5)]:
            c = QtWidgets.QDoubleSpinBox()
            c.setDecimals(3)
            c.setRange(0, maximum)
            c.setSingleStep(.01)
            c.setValue(getattr(self.s, key))
            c.valueChanged.connect(lambda value, k=key: setattr(self.s, k, value))
            self.display_widgets[key] = c
            f.addRow(label, c)
        self.scroll_tab(panel, 'Display overlays')

    def rebuild(self):
        self.gui_generation = self.s.generation
        self.refreshing = True
        self.tree.clear()
        self.items = {}
        def insert(name, parent):
            item = QtWidgets.QTreeWidgetItem([name, '', ''])
            parent.addChild(item) if parent is not None else self.tree.addTopLevelItem(item)
            self.items[name] = item
            item.setCheckState(1, QtCore.Qt.Checked if name in self.s.visible else QtCore.Qt.Unchecked)
            item.setCheckState(2, QtCore.Qt.Checked if name in self.s.members else QtCore.Qt.Unchecked)
            item.setToolTip(0, 'Component: '+self.s.model.root_of(name))
            for j in self.s.model.children.get(name, []):
                insert(self.s.model.joints[j].child, item)
        for root in self.s.model.roots:
            insert(root, None)
        self.tree.expandAll()
        self.tree.setCurrentItem(self.items[self.s.inspected])
        self.refreshing = False
        self.origin_joint.clear()
        self.origin_joint.addItems(sorted(self.s.model.joints))
        self.rebuild_joints()
        self.group_choice.clear()
        self.group_choice.addItems(list(self.rt.presets['presets'].get(self.s.model.variant, {}).get('groups', {})))
        self.load_fields()
        self.joint_panel.setEnabled(self.s.pose_source == 'preview')
        self.edit_panel.setEnabled(self.s.pose_source == 'preview')
        for key,widget in self.overlay_widgets.items():
            widget.blockSignals(True)
            widget.setChecked(self.s.overlays[key])
            widget.blockSignals(False)
        for key,widget in self.display_widgets.items():
            widget.blockSignals(True)
            widget.setValue(getattr(self.s, key))
            widget.blockSignals(False)
        self.filter_tree()
        self.threshold.blockSignals(True)
        self.threshold.setValue(self.s.threshold_m*1000)
        self.threshold.blockSignals(False)

    def filter_tree(self):
        query = self.search.text().lower()
        matches = set()
        for name in self.items:
            if query in name.lower():
                link = name
                matches.add(link)
                while link in self.s.model.parent:
                    link = self.s.model.joints[self.s.model.parent[link]].parent
                    matches.add(link)
        for name, item in self.items.items():
            item.setHidden(name not in matches)

    def select_link(self):
        if self.refreshing or self.tree.currentItem() is None:
            return
        self.s.inspected = self.tree.currentItem().text(0)
        root = self.s.model.root_of(self.s.inspected)
        if self.s.pose_source == 'preview':
            if root != self.s.root:
                self.s.preview_root_pose = transform()
                self.s.pose_note = 'Offline model FK; inspected component root at identity'
        self.s.root = root
        self.load_fields()

    def membership_changed(self, item, column):
        if not self.refreshing and column in (1,2):
            names = self.s.visible if column == 1 else self.s.members
            if item.checkState(column) == QtCore.Qt.Checked:
                names.add(item.text(0))
            else:
                names.discard(item.text(0))

    def start_assembly(self):
        self.s.members = {self.s.inspected}
        self.rebuild()

    def isolate(self):
        self.s.visible = {self.s.inspected}
        self.rebuild()

    def show_all(self):
        self.s.visible = set(self.s.model.links)
        self.rebuild()

    def all_masses(self):
        self.s.members = {n for n,l in self.s.model.links.items() if l.mass is not None}
        self.rebuild()

    def add_group(self):
        group = self.rt.presets['presets'][self.s.model.variant].get('groups', {}).get(self.group_choice.currentText(), [])
        names = set(group)
        if not names <= set(self.s.model.links):
            raise ValueError('Preset group contains unavailable links')
        self.s.members.update(names)
        self.s.visible.update(names)
        self.rebuild()

    def add_adjacent(self, direction):
        model, selected = self.s.model, self.s.inspected
        options = []
        if direction != 'child' and selected in model.parent:
            j = model.joints[model.parent[selected]]
            options.append((j.parent, j.name))
        if direction != 'parent':
            options.extend((model.joints[n].child, n) for n in model.children.get(selected, []))
        if not options:
            raise ValueError('No connecting URDF joints in this direction')
        labels = [n+' via '+j for n,j in options]
        value, ok = QtWidgets.QInputDialog.getItem(self, 'Actual URDF attachment', 'Add module', labels, 0, False)
        if ok:
            target = options[labels.index(value)][0]
            self.s.visible.add(target)
            self.s.members.add(target)
            self.action_status.setText('Attached '+value+'; its mass is included when available.')
            self.rebuild()

    def add_chain(self):
        value, ok = QtWidgets.QInputDialog.getItem(self, 'Actual URDF chain', 'Target link', sorted(self.s.model.links), 0, False)
        if ok:
            path = self.s.add_chain(value, self.chain_masses.isChecked())
            self.action_status.setText('Chain: '+' → '.join(path)+'. Connector masses '+('included' if self.chain_masses.isChecked() else 'unchanged; only target added')+'.')
            self.rebuild()

    def rebuild_joints(self):
        while self.joint_layout.rowCount():
            self.joint_layout.removeRow(0)
        self.joint_widgets = {}
        for name, j in self.s.model.joints.items():
            if j.kind not in ('revolute','continuous','prismatic'):
                continue
            row = QtWidgets.QHBoxLayout()
            slider, number = QtWidgets.QSlider(QtCore.Qt.Horizontal), QtWidgets.QDoubleSpinBox()
            slider.setRange(0, 1000)
            number.setDecimals(6)
            number.setRange(j.lower, j.upper)
            number.setSingleStep(.01)
            q = self.s.model.joint_value(name, self.s.joints)
            number.setValue(q)
            slider.setValue(round(1000*(q-j.lower)/(j.upper-j.lower)) if j.upper > j.lower else 0)
            row.addWidget(slider)
            row.addWidget(number)
            self.joint_widgets[name] = (slider, number)
            def set_q(q, joint=j, sld=slider, num=number):
                if self.s.pose_source != 'preview':
                    return
                sld.blockSignals(True)
                num.blockSignals(True)
                num.setValue(q)
                sld.setValue(round(1000*(q-joint.lower)/(joint.upper-joint.lower)) if joint.upper > joint.lower else 0)
                sld.blockSignals(False)
                num.blockSignals(False)
                self.s.joints[joint.name] = q
            slider.valueChanged.connect(lambda value, fn=set_q, lo=j.lower, hi=j.upper: fn(lo+(hi-lo)*value/1000))
            number.valueChanged.connect(set_q)
            slider.setEnabled(j.mimic is None)
            number.setEnabled(j.mimic is None)
            label = name+' [%s, %s] %s' % (j.lower,j.upper,'m' if j.kind == 'prismatic' else 'rad')
            if j.kind == 'continuous':
                label += ' (UI viewing range only)'
            if j.mimic:
                label += ' mimic '+repr(j.mimic)
            self.joint_layout.addRow(label, row)

    def load_fields(self):
        self.visual_index.blockSignals(True)
        self.visual_index.clear()
        self.visual_index.addItems([str(i)+' '+v.name for i,v in enumerate(self.s.model.links[self.s.inspected].visuals)])
        self.visual_index.blockSignals(False)
        self.load_visual_fields()
        self.load_reference_fields()

    def load_visual_fields(self):
        i = self.visual_index.currentIndex()
        if i < 0 or not hasattr(self, 'visual_xyz'):
            return
        t = self.s.visual_poses.get((self.s.inspected,i), self.s.model.links[self.s.inspected].visuals[i].pose)
        self.visual_xyz.setText(text_vector(t[:3,3]))
        self.visual_rpy.setText(text_vector(rpy_of(t)))

    def apply_visual(self):
        i = self.visual_index.currentIndex()
        if self.s.pose_source != 'preview' or i < 0:
            raise ValueError('Select a visual in preview mode')
        self.s.visual_poses[(self.s.inspected,i)] = transform(read_vector(self.visual_xyz), read_vector(self.visual_rpy))

    def reset_visual(self):
        self.s.visual_poses.pop((self.s.inspected,self.visual_index.currentIndex()), None)
        self.load_visual_fields()

    def load_origin_fields(self):
        name = self.origin_joint.currentText()
        if not name or not hasattr(self, 'origin_xyz'):
            return
        t = self.s.joint_origins.get(name, self.s.model.joints[name].pose)
        self.origin_xyz.setText(text_vector(t[:3,3]))
        self.origin_rpy.setText(text_vector(rpy_of(t)))

    def apply_origin(self):
        if self.s.pose_source != 'preview':
            raise ValueError('Joint origin overrides require preview mode')
        self.s.joint_origins[self.origin_joint.currentText()] = transform(read_vector(self.origin_xyz), read_vector(self.origin_rpy))

    def reset_origin(self):
        self.s.joint_origins.pop(self.origin_joint.currentText(), None)
        self.load_origin_fields()

    def reset_overrides(self):
        self.s.reset_overrides()
        self.rebuild()

    def load_reference_fields(self):
        _, r = self.s.registry.record(self.s.model.variant, self.s.inspected)
        self.ref_ma_known.setChecked(r is not None and r.get('cad_to_mesh') is not None)
        self.ref_ma_xyz.setEnabled(self.ref_ma_known.isChecked())
        self.ref_ma_rpy.setEnabled(self.ref_ma_known.isChecked())
        self.ref_mass.setText('' if r is None else str(r['source']['mass_kg']))
        self.ref_cog.setText('' if r is None else text_vector(r['source']['cog_in_cad_m']))
        self.mass_unit.setCurrentText('kg')
        self.length_unit.setCurrentText('m')
        self.ref_note.setText('' if r is None else r['source'].get('note', ''))
        for prefix, key in [('ma','cad_to_mesh'), ('la','calibrated_cad_to_link')]:
            data = {} if r is None else (r.get(key) or {})
            getattr(self, 'ref_'+prefix+'_xyz').setText(text_vector(data.get('xyz_m', [0,0,0])))
            getattr(self, 'ref_'+prefix+'_rpy').setText(text_vector(data.get('rpy_rad', [0,0,0])))
        for i,key in enumerate(('source','cad_to_mesh','calibrated_cad_to_link')):
            self.ref_statuses[i].setCurrentText('verified' if r is not None and (r.get(key) or {}).get('status') == 'verified' else 'assumed')
        self.ref_complete.setChecked(False if r is None else r['visual'].get('covers_entire_mass_record', False))
        self.ref_follow.setChecked(False if r is None else r.get('allow_follow_current_visual_candidate', False))

    def apply_reference(self):
        i, name = self.visual_index.currentIndex(), self.s.inspected
        if i < 0:
            raise ValueError('A mesh visual is required for CAD registration')
        v = self.s.model.links[name].visuals[i]
        if v.kind != 'mesh':
            raise ValueError('Select a mesh visual for CAD/STL registration')
        m, c, _ = import_properties(float(self.ref_mass.text()), read_vector(self.ref_cog), mass_unit=self.mass_unit.currentText(), length_unit=self.length_unit.currentText())
        data = copy.deepcopy(self.s.registry.data)
        key, old = self.s.registry.record(self.s.model.variant, name)
        key = key or self.s.model.variant+'_'+name
        r = copy.deepcopy(old) if old else {}
        r.update(variant=self.s.model.variant, link=name, instance_id=name, composition_status=r.get('composition_status','unverified'))
        r['visual'] = {'index': i, 'mesh_uri': v.uri, 'mesh_scale': v.size.tolist(), 'mesh_sha256': v.mesh_hash,
                       'origin_fingerprint': v.fingerprint, 'covers_entire_mass_record': self.ref_complete.isChecked()}
        # Editing original properties invalidates an old tensor; it is never copied into a new unknown basis.
        r['source'] = {'kind': 'user_imported_cad', 'status': self.ref_statuses[0].currentText(),
                       'mass_kg': m, 'cog_in_cad_m': c.tolist(), 'inertia_at_cog_kg_m2': None,
                       'note': self.ref_note.text(), 'revision': str(time.time()),
                       'raw_input': {'mass': float(self.ref_mass.text()), 'mass_unit': self.mass_unit.currentText(), 'cog': read_vector(self.ref_cog).tolist(), 'length_unit': self.length_unit.currentText()}}
        for p,key2,status in [('ma','cad_to_mesh',1), ('la','calibrated_cad_to_link',2)]:
            r[key2] = {'xyz_m': read_vector(getattr(self,'ref_'+p+'_xyz')).tolist(), 'rpy_rad': read_vector(getattr(self,'ref_'+p+'_rpy')).tolist(), 'status': self.ref_statuses[status].currentText()}
        if not self.ref_ma_known.isChecked():
            r['cad_to_mesh'] = None
        r['allow_follow_current_visual_candidate'] = self.ref_follow.isChecked()
        r['model_sha256_at_registration'] = self.s.model.fingerprint
        data['records'][key] = r
        self.s.registry = Registry(data)
        self.load_reference_fields()

    def accept_placement(self):
        name = self.s.inspected
        key,r = self.s.registry.record(self.s.model.variant, name)
        if r is None:
            raise ValueError('Import original CAD values and frame relationship first')
        resolved = self.s.registry.resolve(self.s.model, name, self.s.visual_poses)
        if 'candidate' not in resolved:
            raise ValueError('No unambiguous enabled mesh-follow candidate: '+'; '.join(resolved['diagnostics']))
        idx = resolved['visual_index']
        t = self.s.visual_poses.get((name,idx), self.s.model.links[name].visuals[idx].pose) @ placement(r['cad_to_mesh'])
        data = copy.deepcopy(self.s.registry.data)
        data['records'][key]['calibrated_cad_to_link'] = {'xyz_m': t[:3,3].tolist(), 'rpy_rad': rpy_of(t), 'status': 'assumed', 'note': 'Explicitly accepted current mesh placement in viewer'}
        self.s.registry = Registry(data)
        self.load_reference_fields()

    def reset_references(self):
        self.s.registry = Registry(self.s.original_registry)
        self.load_reference_fields()

    def save_reference(self):
        path = self.s.write_export('reference.yaml', yaml.safe_dump(self.s.registry.data, sort_keys=False))
        self.action_status.setText('Saved '+str(path))

    def import_reference(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, 'Import reference registry', str(self.rt.root), 'YAML (*.yaml *.yml)')
        if path:
            registry = Registry.read(path)
            self.s.registry = registry
            self.s.original_registry = copy.deepcopy(registry.data)
            self.load_reference_fields()

    def browse_model(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, 'Model source', str(self.rt.root/'robots'), 'Robot models (*.urdf *.xacro)')
        if path:
            self.source.setText(path)

    def reload_model(self):
        source, preset = self.source.text(), self.preset.currentText()
        try:
            model = load_model(source, preset, self.rt.root, self.rt.presets['presets'][preset]['arguments'])
        except Exception as exc:
            raise ValueError('Reload rejected; retaining last valid generation: '+str(exc)) from exc
        changed_variant = preset != self.s.model.variant
        self.s.replace_model(model)
        self.rt.source, self.rt.preset_name = source, preset
        if changed_variant:
            self.s.members = set(self.rt.presets['presets'][preset].get('initial_members', [])) & set(model.links)
            self.s.visible = set(self.s.members) or set(model.links)
            if self.s.members:
                self.s.inspected = sorted(self.s.members)[0]
                self.s.root = model.root_of(self.s.inspected)
        self.rebuild()

    def set_mode(self):
        self.rt.set_mode(self.pose_mode.currentText(), self.frame.text())
        self.s.tf_prefix = self.prefix.text().strip('/')
        self.rebuild()

    def copy_live(self):
        self.rt.tick()
        self.s.copy_live_snapshot(self.rt.last_poses, self.s.last_report['timestamp'])
        self.rt.mode = 'preview'
        self.pose_mode.setCurrentText('preview')
        self.action_status.setText('Copied live snapshot. Set RViz Fixed Frame to '+self.s.fixed_frame)
        self.rebuild()

    def export(self):
        self.rt.tick()
        path = self.s.export()
        self.action_status.setText('Saved '+str(path))

    def restore(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, 'Load viewer session as offline preview', str(self.s.export_folder()), 'YAML (*.yaml)')
        if path:
            self.s.restore(yaml.safe_load(Path(path).read_text()))
            self.rt.mode = 'preview'
            self.rt.source, self.rt.preset_name = self.s.model.source, self.s.model.variant
            self.source.setText(self.rt.source)
            self.preset.setCurrentText(self.rt.preset_name)
            self.pose_mode.setCurrentText('preview')
            self.prefix.setText(self.s.tf_prefix)
            self.rebuild()

    def tick(self):
        if self.rt.rospy.is_shutdown():
            self.close()
            return
        try:
            report = self.rt.tick()
            if self.gui_generation != self.s.generation:
                self.rebuild()
            s = self.s
            self.banner.setText('%s | %s | frame %s | anchor %s | generation %d | t=%.6f\n%s' %
                                (s.pose_source, s.model.variant, s.fixed_frame, s.root, s.generation, report['timestamp'], s.last_error))
            if time.monotonic()-self.last_audit >= .2:
                # Show an explicit table of source/mass/local/fixed CoG for selected modules,
                # followed by the detailed selected-module record and independent aggregates.
                table_rows = []
                for name in sorted(s.members | {s.inspected}):
                    for mode in ('declared','reference','candidate'):
                        e = report['rows'][name].get(mode)
                        table_rows.append([name, s.model.variant, mode] + (['UNKNOWN']*5 if e is None else
                            ['%.6f' % e['mass_kg'], text_vector(e['cog_local_m']), 'UNAVAILABLE' if e['cog_fixed_m'] is None else text_vector(e['cog_fixed_m']),
                             '—' if 'discrepancy_mm' not in e else '%.3f' % e['discrepancy_mm'], e['label']]) + ['; '.join(report['rows'][name]['diagnostics'])])
                self.audit_table.setRowCount(len(table_rows))
                for row,values in enumerate(table_rows):
                    for col,value in enumerate(values):
                        item = QtWidgets.QTableWidgetItem(str(value))
                        item.setToolTip(str(value))
                        self.audit_table.setItem(row,col,item)
                details = {'pose_note': report['pose_note'], 'inspected': report['rows'][s.inspected], 'aggregates': report['aggregates'], 'all_declared_urdf': report['all_declared_urdf'], 'excluded': report['excluded'], 'diagnostics': report['diagnostics']}
                self.numeric.setPlainText(json.dumps(details, indent=2, allow_nan=False))
                for name,(slider,number) in self.joint_widgets.items():
                    j = s.model.joints[name]
                    if j.mimic and s.pose_source == 'preview':
                        number.setValue(s.model.joint_value(name, s.joints))
                self.last_audit = time.monotonic()
        except Exception as exc:
            self.rt.scene.clear()
            self.banner.setText('Snapshot unavailable: '+str(exc))

    def closeEvent(self, event):
        self.timer.stop()
        self.rt.scene.clear()
        self.rt.rospy.signal_shutdown('Viewer window closed')
        event.accept()


def run(runtime):
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = Window(runtime)
    window.show()
    app.exec_()
