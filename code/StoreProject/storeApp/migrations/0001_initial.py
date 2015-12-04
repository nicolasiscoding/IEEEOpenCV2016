# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
    ]

    operations = [
        migrations.CreateModel(
            name='Order',
            fields=[
                ('order_id', models.AutoField(serialize=False, primary_key=True)),
                ('order_date', models.DateField()),
                ('order_paid', models.BooleanField(default=False)),
            ],
        ),
        migrations.CreateModel(
            name='Product',
            fields=[
                ('product_id', models.AutoField(serialize=False, primary_key=True)),
                ('product_name', models.CharField(max_length=50)),
                ('product_price', models.IntegerField()),
                ('product_stock_quantity', models.IntegerField()),
                ('product_description', models.CharField(max_length=400)),
                ('product_active', models.BooleanField(default=False)),
                ('order', models.ManyToManyField(to='storeApp.Order')),
            ],
        ),
        migrations.CreateModel(
            name='Supplier',
            fields=[
                ('supplier_id', models.AutoField(serialize=False, primary_key=True)),
                ('supplier_name', models.CharField(max_length=100)),
                ('product', models.ManyToManyField(to='storeApp.Product')),
            ],
        ),
        migrations.CreateModel(
            name='User',
            fields=[
                ('user_id', models.AutoField(serialize=False, primary_key=True)),
                ('user_name', models.CharField(max_length=50)),
                ('user_password', models.CharField(max_length=20)),
                ('user_address', models.CharField(max_length=50)),
                ('user_email', models.CharField(max_length=30)),
                ('user_is_staff', models.BooleanField(default=False)),
            ],
        ),
        migrations.AddField(
            model_name='order',
            name='user',
            field=models.ManyToManyField(to='storeApp.User'),
        ),
    ]
