import cgsensor  # �C���|�[�g

bme280 = cgsensor.BME280(i2c_addr=0x76)  # BME280����N���X�̃C���X�^���X, i2c_addr��0x76/0x77����I��

bme280.forced()  # Forced���[�h�ő�����s��, ���ʂ�temperature, pressure, humidity�ɓ����
print('�C�� {}��C'.format(bme280.temperature))  # �C�����擾���ĕ\��
print('���x {}%'.format(bme280.humidity))  # ���x���擾���ĕ\��
print('�C�� {}hPa'.format(bme280.pressure))  # �C�����擾���ĕ\��
