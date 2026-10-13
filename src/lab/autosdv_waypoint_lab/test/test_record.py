import os

from autosdv_waypoint_lab.record import keep_slot, missing_path_file


def test_keep_slot_thins_a_15hz_stream_to_10hz():
    last, kept = None, 0
    for i in range(150):                       # 10 s of 15 Hz arrivals
        keep, slot = keep_slot(i / 15.0, 0.1, last)
        if keep:
            kept += 1
            last = slot
    assert 95 <= kept <= 100


def test_a_survey_only_course_has_no_path_file(tmp_path):
    (tmp_path / 'course.yaml').write_text('markers: []\n')
    assert missing_path_file(str(tmp_path)) == 'path.csv'
    (tmp_path / 'path.csv').write_text('0,0\n')
    assert missing_path_file(str(tmp_path)) is None
